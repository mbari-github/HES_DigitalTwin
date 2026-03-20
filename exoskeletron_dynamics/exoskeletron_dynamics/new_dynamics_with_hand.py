#!/usr/bin/env python3
"""
new_dynamics_with_hand.py
=========================

Nodo ROS2 per la simulazione della dinamica ridotta dell'esoscheletro con modello
della mano (assembly_with_hand.urdf).

Riscrittura completa del nodo originale, con:
- stessa architettura di riduzione dinamica su theta = rev_crank
- stessi topic principali del plant
- stesso layout di /exo_dynamics/debug e /exo_dynamics/ff_terms
- topic aggiuntivo /exo_dynamics/model_debug per diagnostica estesa

Equazione ridotta usata:
    M_eff * theta_ddot =
        tau_m + tau_pass_theta + tau_ext_theta - proj - tau_fric - tau_damp

dove:
    M_eff = B^T M B + Jm
    proj  = B^T (M (Bdot * theta_dot) + h)
    h     = nonLinearEffects(q, dq)
"""

import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, Point
from visualization_msgs.msg import Marker


class ExoReducedDynamicsWithHand(Node):

    def __init__(self) -> None:
        super().__init__('exo_reduced_dynamics_with_hand')

        # ============================================================
        # PARAMETRI ROS2
        # ============================================================

        # --- Percorso URDF e tempi ---
        self.declare_parameter(
            'urdf_path',
            '/home/mbari/ros2_ws/src/exoskeletron_description/urdf/assembly_with_hand.urdf'
        )
        self.declare_parameter('dt', 0.001)
        self.declare_parameter('publish_dt', 0.005)
        self.declare_parameter('theta_init', 0.0)

        # --- Gravità ---
        self.declare_parameter('gravity_zero', False)

        # --- Attriti sul DOF ridotto ---
        self.declare_parameter('motor_inertia', 0.1)
        self.declare_parameter('fric_visc', 0.3)
        self.declare_parameter('fric_coul', 0.3)
        self.declare_parameter('fric_eps', 0.01)
        self.declare_parameter('damping_theta', 0.0)

        # --- Limiti di velocità e accelerazione ---
        self.declare_parameter('max_theta_dot', 10.0)
        self.declare_parameter('max_theta_ddot', 400.0)

        # --- Solver cinematico ---
        self.declare_parameter('closure_tol', 1e-5)
        self.declare_parameter('max_nfev', 150)

        # --- Sicurezza numerica ---
        self.declare_parameter('denom_min', 1e-7)

        # --- Log ---
        self.declare_parameter('log_every_n_steps', 200)

        # --- Bounds su theta ---
        self.declare_parameter('theta_min', -2.5)
        self.declare_parameter('theta_max', 2.5)

        # --- Stato stuck / limit hold ---
        self.declare_parameter('limit_hold', True)
        self.declare_parameter('limit_release_tau', 0.02)
        self.declare_parameter('limit_backoff_step', 0.003)
        self.declare_parameter('limit_backoff_tries', 6)
        self.declare_parameter('limit_use_theta_bounds', True)

        # --- Wrench esterna ---
        self.declare_parameter('external_wrench_enable', True)
        self.declare_parameter('external_wrench_frame', 'frame_CE_end_2')
        self.declare_parameter('external_wrench_topic', '/exo_dynamics/external_wrench')
        self.declare_parameter('external_wrench_scale', 1.0)

        # --- Modello passivo falangi ---
        self.declare_parameter('passive_enable', True)

        # MCF
        self.declare_parameter('K0_MCF', 0.03)
        self.declare_parameter('alpha_MCF', 4.0)
        self.declare_parameter('B_MCF', 0.01)
        self.declare_parameter('rest_MCF', -0.3)

        # IFP
        self.declare_parameter('K0_IFP', 0.03)
        self.declare_parameter('alpha_IFP', 4.0)
        self.declare_parameter('B_IFP', 0.01)
        self.declare_parameter('rest_IFP', -0.5)

        # Saturazioni modello passivo
        self.declare_parameter('passive_K_MAX', 5.0)
        self.declare_parameter('passive_exp_clip', 12.0)

        # --- Soft bound su giunto mediale ---
        self.declare_parameter('medial_soft_enable', True)
        self.declare_parameter('medial_soft_lower', -2.2)
        self.declare_parameter('medial_soft_upper', 0.0)
        self.declare_parameter('medial_soft_weight', 1.0)

        # --- Stima forza CE ---
        self.declare_parameter('ce_force_enable', True)
        self.declare_parameter('ce_force_frame', 'frame_CE_end_2')
        self.declare_parameter('ce_force_marker_scale', 0.02)
        self.declare_parameter('ce_force_marker_diameter', 0.01)

        # ============================================================
        # MODELLO PINOCCHIO
        # ============================================================

        self.urdf_path = str(self.get_parameter('urdf_path').value)
        self.dt = float(self.get_parameter('dt').value)
        self.publish_dt = float(self.get_parameter('publish_dt').value)

        try:
            self.model = pin.buildModelFromUrdf(self.urdf_path)
        except Exception:
            self.model, _, _ = pin.buildModelsFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        self.nq = self.model.nq
        self.nv = self.model.nv

        if bool(self.get_parameter('gravity_zero').value):
            self.model.gravity.linear = np.array([0.0, 0.0, 0.0], dtype=float)
            self.get_logger().warn("gravity_zero=True: gravità disattivata.")

        # --- Giunto attivo ---
        self.jid_crank = self.model.getJointId('rev_crank')
        if self.jid_crank <= 0:
            raise RuntimeError("Joint 'rev_crank' non trovato in URDF.")
        self.idx_theta = self.jid_crank - 1

        # --- Giunti falangi ---
        self.jid_MCF = self.model.getJointId('rev_palmo2prossimale')
        self.jid_IFP = self.model.getJointId('rev_prossimale2mediale')
        if self.jid_MCF <= 0 or self.jid_IFP <= 0:
            raise RuntimeError(
                "Joint mano (rev_palmo2prossimale / rev_prossimale2mediale) non trovato."
            )

        # --- Frame CE ---
        self.ce_force_enable = bool(self.get_parameter('ce_force_enable').value)
        self.ce_force_frame = str(self.get_parameter('ce_force_frame').value)
        self.fid_CE = self.model.getFrameId(self.ce_force_frame)
        if self.ce_force_enable and self.fid_CE < 0:
            self.get_logger().warning(
                f"ce_force_enable=True ma frame '{self.ce_force_frame}' non trovato. Disabilito CE force."
            )
            self.ce_force_enable = False

        # ============================================================
        # FRAME PAIRS PER LA CHIUSURA CINEMATICA
        # ============================================================

        self.closure_frame_name_pairs = [
            ('frame_AC_end', 'frame_rod_end'),
            ('frame_AC_end_2', 'frame_BC_end'),
            ('frame_CE_end', 'frame_BC_end_2'),
            ('frame_CE_end_2', 'slider_t'),
        ]

        self.closure_frame_pairs: List[Tuple[int, int]] = []
        for a, b in self.closure_frame_name_pairs:
            ida = self.model.getFrameId(a)
            idb = self.model.getFrameId(b)
            if ida < 0 or idb < 0:
                self.get_logger().warning(f"Coppia frame non trovata: {a}, {b}")
            else:
                self.closure_frame_pairs.append((ida, idb))

        if len(self.closure_frame_pairs) == 0:
            self.get_logger().warning("Nessuna coppia frame di chiusura valida.")

        # ============================================================
        # DOF DIPENDENTI OTTIMIZZATI DAL SOLVER
        # ============================================================

        dep_joint_names = [
            'rev_body2linkAC',
            'rev_crank2shaft',
            'slider',
            'rev_slider2linkBC',
            'rev_linkAC2linkCE',
            'rev_palmo2prossimale',
            'rev_prossimale2mediale',
            'slider2',
        ]
        dep_ids = [self.model.getJointId(n) for n in dep_joint_names]
        self.idx_opt = [jid - 1 for jid in dep_ids]

        if any(i < 0 for i in self.idx_opt):
            raise RuntimeError("Uno o più joint del subset cinematico non trovati.")

        self.lower = np.array(
            [-2.5, -2.5, -0.015, -2.5, -2.5, -1.5, -2.5, -0.008],
            dtype=float
        )
        self.upper = np.array(
            [2.5, 2.5, 0.004, 2.5, 2.5, 1.2, 2.5, 0.008],
            dtype=float
        )

        # ============================================================
        # STATO RIDOTTO
        # ============================================================

        self.theta = float(self.get_parameter('theta_init').value)
        self.theta_dot = 0.0
        self.theta_ddot = 0.0

        self.q = pin.neutral(self.model).copy()
        self.dq = np.zeros(self.nv)
        self.ddq = np.zeros(self.nv)

        self.last_x = np.zeros(len(self.idx_opt), dtype=float)
        self.B = np.zeros(self.nv)
        self.B_prev = np.zeros(self.nv)

        self.tau_m = 0.0

        # --- Wrench esterna ---
        self.wrench_enable = bool(self.get_parameter('external_wrench_enable').value)
        self.wrench_frame_name = str(self.get_parameter('external_wrench_frame').value)
        self.wrench_topic = str(self.get_parameter('external_wrench_topic').value)
        self.wrench_scale = float(self.get_parameter('external_wrench_scale').value)

        self.fid_ext = -1
        self.wrench_world = np.zeros(6)
        self.tau_ext = np.zeros(self.nv)
        self.tau_ext_theta = 0.0

        if self.wrench_enable:
            self.fid_ext = self.model.getFrameId(self.wrench_frame_name)
            if self.fid_ext < 0:
                self.get_logger().warning(
                    f"external_wrench_enable=True ma frame '{self.wrench_frame_name}' non trovato. Disabilito."
                )
                self.wrench_enable = False

        # --- Passivi ---
        self.passive_enable = bool(self.get_parameter('passive_enable').value)
        self.tau_pass = np.zeros(self.nv)
        self.tau_pass_theta = 0.0

        # --- Diagnostica base ---
        self.step_count = 0
        self.last_solver_success = False
        self.last_solver_nfev = 0
        self.last_closure_norm = np.nan
        self.last_hit_bounds = False

        self.denom_last = 0.0
        self.proj_last = 0.0
        self.gproj_last = 0.0
        self.reaction_theta_last = 0.0
        self.tau_full = np.zeros(self.nv)

        # --- Diagnostica dinamica estesa ---
        self.tau_fric_last = 0.0
        self.tau_damp_last = 0.0
        self.num_last = 0.0
        self.dyn_residual_last = 0.0
        self.tau_model_last = 0.0
        self.tau_model_error_last = 0.0

        # --- Stato finecorsa ---
        self.at_limit = False
        self.limit_dir = 0.0
        self.have_valid = False
        self.theta_valid = float(self.theta)
        self.q_valid = self.q.copy()
        self.dq_valid = self.dq.copy()
        self.B_valid = self.B.copy()
        self.last_x_valid = self.last_x.copy()

        # ============================================================
        # ROS I/O
        # ============================================================

        self.sub_tau = self.create_subscription(Float64, '/torque', self.torque_cb, 10)
        if self.wrench_enable:
            self.sub_wrench = self.create_subscription(
                WrenchStamped, self.wrench_topic, self.wrench_cb, 10
            )

        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_closed = self.create_publisher(JointState, '/joint_states_closed', 10)
        self.pub_dbg = self.create_publisher(Float64MultiArray, '/exo_dynamics/debug', 10)
        self.pub_ff_terms = self.create_publisher(Float64MultiArray, '/exo_dynamics/ff_terms', 10)
        self.pub_tau_ext_theta = self.create_publisher(Float64, '/exo_dynamics/tau_ext_theta', 10)
        self.pub_model_dbg = self.create_publisher(
            Float64MultiArray, '/exo_dynamics/model_debug', 10
        )

        if self.ce_force_enable:
            self.pub_ce_force = self.create_publisher(WrenchStamped, '/ce_force', 10)
            self.pub_ce_marker = self.create_publisher(Marker, '/ce_force_marker', 10)

        # ============================================================
        # INIZIALIZZAZIONE CHIUSURA
        # ============================================================

        q0, _info0 = self.solve_closure(self.theta, update_warmstart=True)
        if q0 is not None:
            self.q = q0
            self.B = self.compute_B(self.q)
            self._store_valid_state()
        else:
            self.get_logger().warning(
                "solve_closure iniziale fallita: partirai in AT_LIMIT finché non rientri."
            )
            self.at_limit = True
            self.theta_dot = 0.0
            self.theta_ddot = 0.0

        self.timer = self.create_timer(self.dt, self.step)
        self.pub_timer = self.create_timer(self.publish_dt, self.publish)

        self.get_logger().info(
            f"ExoReducedDynamicsWithHand avviato | URDF={self.urdf_path} | "
            f"passive_enable={self.passive_enable} | wrench_enable={self.wrench_enable}"
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def torque_cb(self, msg: Float64) -> None:
        self.tau_m = float(msg.data)

    def wrench_cb(self, msg: WrenchStamped) -> None:
        s = self.wrench_scale
        self.wrench_world = np.array(
            [
                s * msg.wrench.force.x,
                s * msg.wrench.force.y,
                s * msg.wrench.force.z,
                s * msg.wrench.torque.x,
                s * msg.wrench.torque.y,
                s * msg.wrench.torque.z,
            ],
            dtype=float,
        )

    # ============================================================
    # HELPERS
    # ============================================================

    def _store_valid_state(self) -> None:
        self.have_valid = True
        self.theta_valid = float(self.theta)
        self.q_valid = self.q.copy()
        self.dq_valid = self.dq.copy()
        self.B_valid = self.B.copy()
        self.last_x_valid = self.last_x.copy()

    # ============================================================
    # CHIUSURA CINEMATICA
    # ============================================================

    def closure_error(self, q: np.ndarray) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        e: List[float] = []
        for ida, idb in self.closure_frame_pairs:
            e.extend(self.data.oMf[ida].translation - self.data.oMf[idb].translation)
        return np.array(e, dtype=float)

    def solve_closure(
        self,
        theta: float,
        update_warmstart: bool,
    ) -> Tuple[Optional[np.ndarray], Dict[str, float]]:
        info: Dict[str, float] = {
            'success': False,
            'nfev': 0,
            'closure_norm': np.inf,
            'hit_bounds': False,
        }

        q = self.q.copy()
        q[self.idx_theta] = theta

        if len(self.closure_frame_pairs) == 0:
            cnorm = float(np.linalg.norm(self.closure_error(q)))
            info['success'] = True
            info['nfev'] = 0
            info['closure_norm'] = cnorm
            return q, info

        medial_soft_enable = bool(self.get_parameter('medial_soft_enable').value)
        medial_lower = float(self.get_parameter('medial_soft_lower').value)
        medial_upper = float(self.get_parameter('medial_soft_upper').value)
        medial_w = float(self.get_parameter('medial_soft_weight').value)
        idx_medial_in_x = 6

        def residuals(x: np.ndarray) -> np.ndarray:
            q_loc = q.copy()
            for k, idx in enumerate(self.idx_opt):
                q_loc[idx] = x[k]
            e = self.closure_error(q_loc)

            if not medial_soft_enable:
                return e

            medial = float(x[idx_medial_in_x])
            penalty = 0.0
            if medial < medial_lower:
                penalty += (medial - medial_lower) ** 2
            if medial > medial_upper:
                penalty += (medial - medial_upper) ** 2

            return np.hstack([e, math.sqrt(medial_w * penalty)])

        sol = least_squares(
            residuals,
            self.last_x.copy(),
            method='trf',
            bounds=(self.lower, self.upper),
            xtol=1e-8,
            ftol=1e-8,
            gtol=1e-8,
            max_nfev=int(self.get_parameter('max_nfev').value),
        )

        info['success'] = bool(sol.success)
        info['nfev'] = int(sol.nfev)

        epsb = 1e-6
        info['hit_bounds'] = any(
            abs(sol.x[k] - self.lower[k]) < epsb or abs(sol.x[k] - self.upper[k]) < epsb
            for k in range(len(sol.x))
        )

        if not sol.success:
            return None, info

        q_sol = q.copy()
        for k, idx in enumerate(self.idx_opt):
            q_sol[idx] = sol.x[k]

        cnorm = float(np.linalg.norm(self.closure_error(q_sol)))
        info['closure_norm'] = cnorm

        tol = float(self.get_parameter('closure_tol').value)
        if cnorm > tol:
            info['success'] = False
            return None, info

        if update_warmstart:
            self.last_x = sol.x.copy()

        return q_sol, info

    # ============================================================
    # VETTORE DI PROIEZIONE B = dq/dtheta
    # ============================================================

    def compute_B(self, q: np.ndarray) -> np.ndarray:
        if len(self.closure_frame_pairs) == 0:
            B = np.zeros(self.nv)
            B[self.idx_theta] = 1.0
            return B

        A_rows: List[np.ndarray] = []
        for ida, idb in self.closure_frame_pairs:
            JA = pin.computeFrameJacobian(
                self.model, self.data, q, ida, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )[:3, :]
            JB = pin.computeFrameJacobian(
                self.model, self.data, q, idb, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )[:3, :]
            A_rows.append(JA - JB)

        A = np.vstack(A_rows)

        e_theta = np.zeros(self.nv)
        e_theta[self.idx_theta] = 1.0
        rhs = -A @ e_theta

        mask = np.ones(self.nv, dtype=bool)
        mask[self.idx_theta] = False
        A_red = A[:, mask]

        x_red, *_ = np.linalg.lstsq(A_red, rhs, rcond=None)
        x = np.zeros(self.nv)
        x[mask] = x_red
        return e_theta + x

    # ============================================================
    # WRENCH ESTERNA
    # ============================================================

    def compute_tau_ext(self, q: np.ndarray) -> None:
        if (not self.wrench_enable) or (self.fid_ext < 0):
            self.tau_ext[:] = 0.0
            self.tau_ext_theta = 0.0
            return

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        J6 = pin.computeFrameJacobian(
            self.model, self.data, q, self.fid_ext, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        W = pin.Force(self.wrench_world)
        self.tau_ext = (J6.T @ W.vector).copy()
        self.tau_ext_theta = float(self.B.T @ self.tau_ext)

    # ============================================================
    # TORQUES PASSIVI
    # ============================================================

    def compute_passive_torques(self, q: np.ndarray, dq: np.ndarray) -> None:
        self.tau_pass[:] = 0.0
        self.tau_pass_theta = 0.0
        if not self.passive_enable:
            return

        K_MAX = float(self.get_parameter('passive_K_MAX').value)
        exp_clip = float(self.get_parameter('passive_exp_clip').value)

        def safe_exp_stiffness(K0: float, alpha: float, error: float) -> float:
            x = alpha * abs(error)
            if x > exp_clip:
                return K_MAX
            return min(K0 * math.exp(x), K_MAX)

        idx_MCF = self.jid_MCF - 1
        idx_IFP = self.jid_IFP - 1

        q_MCF = float(q[idx_MCF])
        dq_MCF = float(dq[idx_MCF])
        rest_MCF = float(self.get_parameter('rest_MCF').value)
        err_MCF = q_MCF - rest_MCF
        K_MCF = safe_exp_stiffness(
            float(self.get_parameter('K0_MCF').value),
            float(self.get_parameter('alpha_MCF').value),
            err_MCF,
        )
        B_MCF = float(self.get_parameter('B_MCF').value)
        tau_MCF = -K_MCF * err_MCF - B_MCF * dq_MCF

        q_IFP = float(q[idx_IFP])
        dq_IFP = float(dq[idx_IFP])
        rest_IFP = float(self.get_parameter('rest_IFP').value)
        err_IFP = q_IFP - rest_IFP
        K_IFP = safe_exp_stiffness(
            float(self.get_parameter('K0_IFP').value),
            float(self.get_parameter('alpha_IFP').value),
            err_IFP,
        )
        B_IFP = float(self.get_parameter('B_IFP').value)
        tau_IFP = -K_IFP * err_IFP - B_IFP * dq_IFP

        self.tau_pass[idx_MCF] = tau_MCF
        self.tau_pass[idx_IFP] = tau_IFP

        self.tau_pass_theta = float(self.B.T @ self.tau_pass)

    # ============================================================
    # FINECORSA / STUCK
    # ============================================================

    def _theta_within_bounds(self, theta: float) -> bool:
        if not bool(self.get_parameter('limit_use_theta_bounds').value):
            return True
        theta_min = float(self.get_parameter('theta_min').value)
        theta_max = float(self.get_parameter('theta_max').value)
        return theta_min <= theta <= theta_max

    def _clamp_theta_bounds(self, theta: float) -> float:
        if not bool(self.get_parameter('limit_use_theta_bounds').value):
            return float(theta)
        theta_min = float(self.get_parameter('theta_min').value)
        theta_max = float(self.get_parameter('theta_max').value)
        return float(np.clip(theta, theta_min, theta_max))

    def _stuck_try_release(self) -> bool:
        if not self.at_limit or (not self.have_valid):
            return False

        tau_release = float(self.get_parameter('limit_release_tau').value)

        theta_dot = self.theta_dot
        b = float(self.get_parameter('fric_visc').value)
        fc = float(self.get_parameter('fric_coul').value)
        eps = float(self.get_parameter('fric_eps').value)
        tau_fric = b * theta_dot + fc * np.tanh(theta_dot / max(eps, 1e-9))

        gproj = float(self.gproj_last)
        tau_eff = (
            self.tau_m
            + float(self.tau_ext_theta)
            + float(self.tau_pass_theta)
            - tau_fric
            - gproj
        )

        if (tau_eff * self.limit_dir) >= -tau_release:
            return False

        base_step = float(self.get_parameter('limit_backoff_step').value)
        tries = int(self.get_parameter('limit_backoff_tries').value)

        for k in range(1, tries + 1):
            th_try = self.theta_valid - self.limit_dir * (base_step * k)
            th_try = self._clamp_theta_bounds(th_try)

            self.q = self.q_valid.copy()
            self.last_x = self.last_x_valid.copy()

            q_try, _ = self.solve_closure(th_try, update_warmstart=False)
            if q_try is None:
                continue

            self.theta = float(th_try)
            self.q = q_try
            self.B = self.compute_B(self.q)
            self.theta_dot = 0.0
            self.theta_ddot = 0.0
            self.dq[:] = 0.0
            self.ddq[:] = 0.0
            self.at_limit = False
            self._store_valid_state()
            return True

        return False

    # ============================================================
    # STEP DINAMICO
    # ============================================================

    def step(self) -> None:
        self.step_count += 1
        logN = max(1, int(self.get_parameter('log_every_n_steps').value))

        # 0) Se sono in AT_LIMIT, provo a rilasciare
        if self.at_limit:
            released = self._stuck_try_release()
            if not released:
                self.dq[:] = 0.0
                self.ddq[:] = 0.0
                return

        # 1) Theta entro bounds
        self.theta = self._clamp_theta_bounds(self.theta)

        # 2) Risolvi chiusura cinematica per theta corrente
        q_new, info = self.solve_closure(self.theta, update_warmstart=True)
        self.last_solver_success = bool(info['success'])
        self.last_solver_nfev = int(info['nfev'])
        self.last_closure_norm = float(info['closure_norm'])
        self.last_hit_bounds = bool(info['hit_bounds'])

        if q_new is None:
            self.at_limit = True
            self.limit_dir = float(np.sign(self.theta_dot)) if abs(self.theta_dot) > 1e-9 else 1.0
            self.theta = float(self.theta_valid)
            self.q = self.q_valid.copy()
            self.B = self.B_valid.copy()
            self.last_x = self.last_x_valid.copy()
            self.theta_dot = 0.0
            self.theta_ddot = 0.0
            self.dq[:] = 0.0
            self.ddq[:] = 0.0

            if self.step_count % logN == 0:
                self.get_logger().warning(
                    f"[step {self.step_count}] solve_closure fallita -> AT_LIMIT | "
                    f"closure_norm={self.last_closure_norm:.3e} nfev={self.last_solver_nfev} "
                    f"hit_bounds={self.last_hit_bounds}"
                )
            return

        # 3) Aggiorna cinematica e B
        self.q = q_new
        self.B_prev = self.B.copy()
        self.B = self.compute_B(self.q)
        self.at_limit = False
        self._store_valid_state()

        Bdot = (self.B - self.B_prev) / self.dt

        # 4) dq, ddq preliminari
        self.dq = self.B * self.theta_dot
        self.ddq = self.B * self.theta_ddot + Bdot * self.theta_dot

        # 5) Dinamica completa
        M = pin.crba(self.model, self.data, self.q)
        h = pin.nonLinearEffects(self.model, self.data, self.q, self.dq)

        # 6) Ingressi esterni e passivi
        self.compute_tau_ext(self.q)
        self.compute_passive_torques(self.q, self.dq)

        # 7) Scalari ridotti
        denom_mech = float(self.B.T @ (M @ self.B))
        Jm = float(self.get_parameter('motor_inertia').value)
        denom = denom_mech + Jm
        self.denom_last = denom

        self.gproj_last = float(self.B.T @ h)
        self.proj_last = float(self.B.T @ (M @ (Bdot * self.theta_dot) + h))

        denom_min = float(self.get_parameter('denom_min').value)
        if abs(denom) < denom_min:
            self.at_limit = True
            self.limit_dir = float(np.sign(self.theta_dot)) if abs(self.theta_dot) > 1e-6 else 1.0
            self.theta = float(self.theta_valid)
            self.q = self.q_valid.copy()
            self.B = self.B_valid.copy()
            self.last_x = self.last_x_valid.copy()
            self.theta_dot = 0.0
            self.theta_ddot = 0.0
            self.dq[:] = 0.0
            self.ddq[:] = 0.0

            if self.step_count % logN == 0:
                self.get_logger().warning(
                    f"[step {self.step_count}] denom TOO SMALL -> AT_LIMIT | "
                    f"denom={denom:.3e} (mech={denom_mech:.3e}+Jm={Jm:.3e})"
                )
            return

        # 8) Attrito + damping
        b = float(self.get_parameter('fric_visc').value)
        fc = float(self.get_parameter('fric_coul').value)
        eps = float(self.get_parameter('fric_eps').value)

        tau_fric = b * self.theta_dot + fc * np.tanh(self.theta_dot / max(eps, 1e-9))

        damping_theta = float(self.get_parameter('damping_theta').value)
        tau_damp = damping_theta * self.theta_dot

        # 9) Equazione del moto scalare
        num = (
            self.tau_m
            + self.tau_pass_theta
            + self.tau_ext_theta
        ) - self.proj_last - tau_fric - tau_damp

        self.theta_ddot = num / denom
        max_dd = float(self.get_parameter('max_theta_ddot').value)
        self.theta_ddot = float(np.clip(self.theta_ddot, -max_dd, max_dd))

        # 10) Diagnostica estesa
        self.tau_fric_last = float(tau_fric)
        self.tau_damp_last = float(tau_damp)
        self.num_last = float(num)

        self.dyn_residual_last = float(self.denom_last * self.theta_ddot - self.num_last)

        self.tau_model_last = float(
            self.denom_last * self.theta_ddot
            - self.tau_ext_theta
            - self.tau_pass_theta
            + self.proj_last
            + self.tau_fric_last
            + self.tau_damp_last
        )

        self.tau_model_error_last = float(self.tau_m - self.tau_model_last)

        # 11) Integrazione Eulero esplicito
        self.theta_dot += self.theta_ddot * self.dt
        max_d = float(self.get_parameter('max_theta_dot').value)
        self.theta_dot = float(np.clip(self.theta_dot, -max_d, max_d))

        theta_next = self.theta + self.theta_dot * self.dt
        theta_next = self._clamp_theta_bounds(theta_next)

        if bool(self.get_parameter('limit_hold').value):
            if (abs(theta_next - self.theta) < 1e-12) and (abs(self.theta_dot) > 1e-8):
                self.at_limit = True
                self.limit_dir = (
                    float(np.sign(self.theta_dot)) if abs(self.theta_dot) > 1e-9 else 1.0
                )
                self.theta_dot = 0.0
                self.theta_ddot = 0.0
                self.theta = float(theta_next)
                self._store_valid_state()
                return

        self.theta = float(theta_next)

        # 12) dq, ddq finali
        self.dq = self.B * self.theta_dot
        self.ddq = self.B * self.theta_ddot + Bdot * self.theta_dot

        # 13) Tau full da RNEA
        tau_rnea = pin.rnea(self.model, self.data, self.q, self.dq, self.ddq)
        self.tau_full = (tau_rnea - self.tau_ext - self.tau_pass).copy()

        # 14) Reazione equivalente su theta
        self.reaction_theta_last = float(
            self.proj_last + tau_fric + tau_damp
            - (self.tau_m + self.tau_pass_theta + self.tau_ext_theta)
        )

        # Logging opzionale
        # if self.step_count % logN == 0:
        #     self.get_logger().info(
        #         f"[step {self.step_count}] theta={self.theta:.4f} thdot={self.theta_dot:.4f} "
        #         f"thddot={self.theta_ddot:.2f} | "
        #         f"tau={self.tau_m:.3f} tauPassθ={self.tau_pass_theta:.3f} "
        #         f"tauExtθ={self.tau_ext_theta:.3f} | "
        #         f"denom={self.denom_last:.3e} (Jm={Jm:.3e}) | "
        #         f"gproj={self.gproj_last:.4f} proj={self.proj_last:.4f} | "
        #         f"closure_norm={self.last_closure_norm:.3e} nfev={self.last_solver_nfev} "
        #         f"hit_bounds={self.last_hit_bounds} at_limit={self.at_limit}"
        #     )

    # ============================================================
    # STIMA FORZA AL PUNTO CE
    # ============================================================

    def _publish_ce_force(self) -> None:
        if not self.ce_force_enable:
            return

        pin.computeJointJacobians(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        J6 = pin.computeFrameJacobian(
            self.model,
            self.data,
            self.q,
            self.fid_CE,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        Jv = J6[:3, :]

        try:
            F = np.linalg.pinv(Jv.T) @ self.tau_full
        except Exception:
            F = np.zeros(3)

        now = self.get_clock().now().to_msg()

        w = WrenchStamped()
        w.header.stamp = now
        w.header.frame_id = self.ce_force_frame
        w.wrench.force.x = float(F[0])
        w.wrench.force.y = float(F[1])
        w.wrench.force.z = float(F[2])
        self.pub_ce_force.publish(w)

        m = Marker()
        m.header.frame_id = self.ce_force_frame
        m.header.stamp = now
        m.ns = 'ce_force'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD

        scale = float(self.get_parameter('ce_force_marker_scale').value)
        m.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=float(scale * F[0]), y=float(scale * F[1]), z=float(scale * F[2])),
        ]
        d = float(self.get_parameter('ce_force_marker_diameter').value)
        m.scale.x = d
        m.scale.y = 2.0 * d
        m.scale.z = 2.0 * d
        m.color.r = 1.0
        m.color.a = 1.0
        self.pub_ce_marker.publish(m)

    # ============================================================
    # PUBLISH
    # ============================================================

    def publish(self) -> None:
        # --- /joint_states (subset) ---
        mech_names = [
            'rev_crank',
            'rev_body2linkAC',
            'rev_crank2shaft',
            'slider',
            'rev_slider2linkBC',
            'rev_linkAC2linkCE',
            'rev_palmo2prossimale',
            'rev_prossimale2mediale',
            'slider2',
        ]
        ids = [self.model.getJointId(n) for n in mech_names]
        idxs = [jid - 1 for jid in ids]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = mech_names
        js.position = [float(self.q[i]) for i in idxs]
        js.velocity = [float(self.dq[i]) for i in idxs]
        js.effort = [float(self.tau_full[i]) for i in idxs]
        self.pub_js.publish(js)

        # --- /joint_states_closed ---
        js2 = JointState()
        js2.header.stamp = js.header.stamp
        js2.name = self.model.names[1:]
        js2.position = self.q.tolist()
        js2.velocity = self.dq.tolist()
        js2.effort = self.tau_full.tolist()
        self.pub_closed.publish(js2)

        # --- /exo_dynamics/debug ---
        dbg = Float64MultiArray()
        dbg.data = [
            float(self.theta),
            float(self.theta_dot),
            float(self.theta_ddot),
            float(self.tau_m),
            float(self.denom_last),
            float(self.proj_last),
            float(self.gproj_last),
            float(self.last_closure_norm if np.isfinite(self.last_closure_norm) else -1.0),
            1.0 if self.last_solver_success else 0.0,
            float(self.last_solver_nfev),
            1.0 if self.last_hit_bounds else 0.0,
            1.0 if self.at_limit else 0.0,
            float(self.tau_ext_theta),
            float(self.tau_pass_theta),
            float(self.reaction_theta_last),
        ]
        self.pub_dbg.publish(dbg)

        # --- /exo_dynamics/ff_terms ---
        ff = Float64MultiArray()
        ff.data = [
            float(self.denom_last),
            float(self.proj_last),
            float(self.gproj_last),
            float(self.tau_pass_theta),
        ]
        self.pub_ff_terms.publish(ff)

        # --- /exo_dynamics/model_debug ---
        # Layout:
        # [0]  theta
        # [1]  theta_dot
        # [2]  theta_ddot
        # [3]  tau_m
        # [4]  tau_ext_theta
        # [5]  tau_pass_theta
        # [6]  M_eff
        # [7]  proj
        # [8]  g_proj
        # [9]  tau_fric
        # [10] tau_damp
        # [11] dyn_num
        # [12] dyn_residual
        # [13] tau_model
        # [14] tau_model_error
        model_dbg = Float64MultiArray()
        model_dbg.data = [
            float(self.theta),
            float(self.theta_dot),
            float(self.theta_ddot),
            float(self.tau_m),
            float(self.tau_ext_theta),
            float(self.tau_pass_theta),
            float(self.denom_last),
            float(self.proj_last),
            float(self.gproj_last),
            float(self.tau_fric_last),
            float(self.tau_damp_last),
            float(self.num_last),
            float(self.dyn_residual_last),
            float(self.tau_model_last),
            float(self.tau_model_error_last),
        ]
        self.pub_model_dbg.publish(model_dbg)

        # --- /exo_dynamics/tau_ext_theta ---
        msg_tau_ext = Float64()
        msg_tau_ext.data = float(self.tau_ext_theta)
        self.pub_tau_ext_theta.publish(msg_tau_ext)

        # --- /ce_force + marker ---
        self._publish_ce_force()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExoReducedDynamicsWithHand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()