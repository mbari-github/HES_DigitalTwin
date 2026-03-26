#!/usr/bin/env python3
"""
dynamics_control_test.py
=========================
Nodo ROS 2 di dinamica ridotta dell'esoscheletro — versione semplificata
per il test e la validazione degli anelli di controllo.

Rispetto a new_dynamics_with_hand.py, questa versione:
  - rimuove la wrench esterna cartesiana (niente WrenchStamped, Jacobiano al
    contact frame, Marker, CE force estimation)
  - accetta direttamente un override scalare su tau_ext_theta via Float64
    sul topic /exo_dynamics/tau_ext_theta_override
  - rimuove il topic /exo_dynamics/model_debug (diagnostica estesa)

Tutto il core è identico: chiusura cinematica, riduzione B, CRBA/RNEA,
coppie passive Fung, attrito, integrazione Euler, ff_terms.

Equazione ridotta:
    M_eff * theta_ddot =
        tau_m + tau_pass_theta + tau_ext_theta - proj - tau_fric - tau_damp

Topics pubblicati:
    /joint_states                   sensor_msgs/JointState
    /exo_dynamics/debug             std_msgs/Float64MultiArray
    /exo_dynamics/ff_terms          std_msgs/Float64MultiArray
    /exo_dynamics/tau_ext_theta     std_msgs/Float64

Topics sottoscritti:
    /torque                                 std_msgs/Float64
    /exo_dynamics/tau_ext_theta_override    std_msgs/Float64
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


class ExoDynamicsControlTest(Node):

    def __init__(self) -> None:
        super().__init__('exo_dynamics_control_test')

        # ============================================================
        # PARAMETRI ROS 2
        # ============================================================

        self.declare_parameter(
            'urdf_path',
            '/home/mbari/ros2_ws/src/HES_DigitalTwin/'
            'exoskeletron_description/urdf/assembly_with_hand.urdf'
        )
        self.declare_parameter('dt', 0.001)
        self.declare_parameter('publish_dt', 0.005)
        self.declare_parameter('theta_init', 0.0)

        self.declare_parameter('gravity_zero', False)

        self.declare_parameter('motor_inertia', 0.1)
        self.declare_parameter('fric_visc', 0.3)
        self.declare_parameter('fric_coul', 0.3)
        self.declare_parameter('fric_eps', 0.01)
        self.declare_parameter('damping_theta', 0.0)

        self.declare_parameter('max_theta_dot', 10.0)
        self.declare_parameter('max_theta_ddot', 400.0)

        self.declare_parameter('closure_tol', 1e-5)
        self.declare_parameter('max_nfev', 150)
        self.declare_parameter('denom_min', 1e-7)
        self.declare_parameter('log_every_n_steps', 200)

        self.declare_parameter('theta_min', -2.5)
        self.declare_parameter('theta_max', 2.5)
        self.declare_parameter('limit_hold', True)
        self.declare_parameter('limit_release_tau', 0.02)
        self.declare_parameter('limit_backoff_step', 0.003)
        self.declare_parameter('limit_backoff_tries', 6)
        self.declare_parameter('limit_use_theta_bounds', True)

        # --- Modello passivo falangi ---
        self.declare_parameter('passive_enable', True)
        self.declare_parameter('K0_MCF', 0.03)
        self.declare_parameter('alpha_MCF', 4.0)
        self.declare_parameter('B_MCF', 0.01)
        self.declare_parameter('rest_MCF', -0.3)
        self.declare_parameter('K0_IFP', 0.03)
        self.declare_parameter('alpha_IFP', 4.0)
        self.declare_parameter('B_IFP', 0.01)
        self.declare_parameter('rest_IFP', -0.5)
        self.declare_parameter('passive_K_MAX', 5.0)
        self.declare_parameter('passive_exp_clip', 12.0)

        # --- Soft bound su giunto mediale ---
        self.declare_parameter('medial_soft_enable', True)
        self.declare_parameter('medial_soft_lower', -2.2)
        self.declare_parameter('medial_soft_upper', 0.0)
        self.declare_parameter('medial_soft_weight', 1.0)

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
            self.model.gravity.linear = np.array([0.0, 0.0, 0.0])
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
                "Joint mano (rev_palmo2prossimale / rev_prossimale2mediale) "
                "non trovato."
            )

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
                self.get_logger().warning(
                    f"Coppia frame non trovata: {a}, {b}"
                )
            else:
                self.closure_frame_pairs.append((ida, idb))

        # ============================================================
        # DOF DIPENDENTI
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
            raise RuntimeError(
                "Uno o più joint del subset cinematico non trovati."
            )

        self.lower = np.array(
            [-2.5, -2.5, -0.015, -2.5, -2.5, -1.5, -2.5, -0.008]
        )
        self.upper = np.array(
            [2.5, 2.5, 0.004, 2.5, 2.5, 1.2, 2.5, 0.008]
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

        self.last_x = np.zeros(len(self.idx_opt))
        self.B = np.zeros(self.nv)
        self.B_prev = np.zeros(self.nv)

        self.tau_m = 0.0

        # --- Override diretto di tau_ext_theta ---
        self.tau_ext = np.zeros(self.nv)
        self.tau_ext_theta = 0.0
        self._tau_ext_override_value = 0.0

        # --- Passivi ---
        self.passive_enable = bool(self.get_parameter('passive_enable').value)
        self.tau_pass = np.zeros(self.nv)
        self.tau_pass_theta = 0.0

        # --- Diagnostica ---
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
        self.tau_fric_last = 0.0
        self.tau_damp_last = 0.0
        self.num_last = 0.0

        # --- Finecorsa ---
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

        self.sub_tau = self.create_subscription(
            Float64, '/torque', self._torque_cb, 10
        )
        self.sub_tau_ext_override = self.create_subscription(
            Float64,
            '/exo_dynamics/tau_ext_theta_override',
            self._tau_ext_override_cb,
            10
        )

        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_dbg = self.create_publisher(
            Float64MultiArray, '/exo_dynamics/debug', 10
        )
        self.pub_ff_terms = self.create_publisher(
            Float64MultiArray, '/exo_dynamics/ff_terms', 10
        )
        self.pub_tau_ext_theta = self.create_publisher(
            Float64, '/exo_dynamics/tau_ext_theta', 10
        )

        # ============================================================
        # INIZIALIZZAZIONE CHIUSURA
        # ============================================================

        q0, _ = self.solve_closure(self.theta, update_warmstart=True)
        if q0 is not None:
            self.q = q0
            self.B = self.compute_B(self.q)
            self._store_valid_state()
        else:
            self.get_logger().warning(
                "solve_closure iniziale fallita: AT_LIMIT."
            )
            self.at_limit = True
            self.theta_dot = 0.0
            self.theta_ddot = 0.0

        self.timer = self.create_timer(self.dt, self.step)
        self.pub_timer = self.create_timer(self.publish_dt, self.publish)

        self.get_logger().info(
            f"ExoDynamicsControlTest avviato | URDF={self.urdf_path} | "
            f"passive_enable={self.passive_enable} | "
            f"tau_ext_theta override ENABLED"
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def _torque_cb(self, msg: Float64) -> None:
        self.tau_m = float(msg.data)

    def _tau_ext_override_cb(self, msg: Float64) -> None:
        self._tau_ext_override_value = float(msg.data)

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
            e.extend(
                self.data.oMf[ida].translation
                - self.data.oMf[idb].translation
            )
        return np.array(e, dtype=float)

    def solve_closure(
        self, theta: float, update_warmstart: bool,
    ) -> Tuple[Optional[np.ndarray], Dict[str, float]]:
        info: Dict[str, float] = {
            'success': False, 'nfev': 0,
            'closure_norm': np.inf, 'hit_bounds': False,
        }

        q = self.q.copy()
        q[self.idx_theta] = theta

        if len(self.closure_frame_pairs) == 0:
            cnorm = float(np.linalg.norm(self.closure_error(q)))
            info.update(success=True, nfev=0, closure_norm=cnorm)
            return q, info

        medial_soft_enable = bool(
            self.get_parameter('medial_soft_enable').value
        )
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
            residuals, self.last_x.copy(), method='trf',
            bounds=(self.lower, self.upper),
            xtol=1e-8, ftol=1e-8, gtol=1e-8,
            max_nfev=int(self.get_parameter('max_nfev').value),
        )

        info['success'] = bool(sol.success)
        info['nfev'] = int(sol.nfev)
        epsb = 1e-6
        info['hit_bounds'] = any(
            abs(sol.x[k] - self.lower[k]) < epsb
            or abs(sol.x[k] - self.upper[k]) < epsb
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
    # PROIEZIONE B = dq/dtheta
    # ============================================================

    def compute_B(self, q: np.ndarray) -> np.ndarray:
        if len(self.closure_frame_pairs) == 0:
            B = np.zeros(self.nv)
            B[self.idx_theta] = 1.0
            return B

        A_rows: List[np.ndarray] = []
        for ida, idb in self.closure_frame_pairs:
            JA = pin.computeFrameJacobian(
                self.model, self.data, q, ida,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )[:3, :]
            JB = pin.computeFrameJacobian(
                self.model, self.data, q, idb,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
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
    # TAU_EXT (override diretto)
    # ============================================================

    def compute_tau_ext(self) -> None:
        self.tau_ext[:] = 0.0
        self.tau_ext_theta = self._tau_ext_override_value

    # ============================================================
    # TORQUES PASSIVI
    # ============================================================

    def compute_passive_torques(
        self, q: np.ndarray, dq: np.ndarray
    ) -> None:
        self.tau_pass[:] = 0.0
        self.tau_pass_theta = 0.0
        if not self.passive_enable:
            return

        K_MAX = float(self.get_parameter('passive_K_MAX').value)
        exp_clip = float(self.get_parameter('passive_exp_clip').value)

        def safe_exp_stiffness(K0, alpha, error):
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
    # FINECORSA
    # ============================================================

    def _clamp_theta_bounds(self, theta: float) -> float:
        if not bool(self.get_parameter('limit_use_theta_bounds').value):
            return float(theta)
        return float(np.clip(
            theta,
            float(self.get_parameter('theta_min').value),
            float(self.get_parameter('theta_max').value),
        ))

    def _stuck_try_release(self) -> bool:
        if not self.at_limit or not self.have_valid:
            return False

        tau_release = float(self.get_parameter('limit_release_tau').value)
        b = float(self.get_parameter('fric_visc').value)
        fc = float(self.get_parameter('fric_coul').value)
        eps = float(self.get_parameter('fric_eps').value)
        tau_fric = (
            b * self.theta_dot
            + fc * np.tanh(self.theta_dot / max(eps, 1e-9))
        )
        gproj = float(self.gproj_last)
        tau_eff = (
            self.tau_m
            + self.tau_ext_theta
            + self.tau_pass_theta
            - tau_fric - gproj
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

        # 0) Se in AT_LIMIT, prova release
        if self.at_limit:
            if not self._stuck_try_release():
                self.dq[:] = 0.0
                self.ddq[:] = 0.0
                return

        # 1) Clamp theta
        self.theta = self._clamp_theta_bounds(self.theta)

        # 2) Chiusura cinematica
        q_new, info = self.solve_closure(self.theta, update_warmstart=True)
        self.last_solver_success = bool(info['success'])
        self.last_solver_nfev = int(info['nfev'])
        self.last_closure_norm = float(info['closure_norm'])
        self.last_hit_bounds = bool(info['hit_bounds'])

        if q_new is None:
            self.at_limit = True
            self.limit_dir = (
                float(np.sign(self.theta_dot))
                if abs(self.theta_dot) > 1e-9 else 1.0
            )
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
                    f"[step {self.step_count}] solve_closure fallita "
                    f"-> AT_LIMIT"
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
        self.compute_tau_ext()
        self.compute_passive_torques(self.q, self.dq)

        # 7) Scalari ridotti
        denom_mech = float(self.B.T @ (M @ self.B))
        Jm = float(self.get_parameter('motor_inertia').value)
        denom = denom_mech + Jm
        self.denom_last = denom

        self.gproj_last = float(self.B.T @ h)
        self.proj_last = float(
            self.B.T @ (M @ (Bdot * self.theta_dot) + h)
        )

        denom_min = float(self.get_parameter('denom_min').value)
        if abs(denom) < denom_min:
            self.at_limit = True
            self.limit_dir = (
                float(np.sign(self.theta_dot))
                if abs(self.theta_dot) > 1e-6 else 1.0
            )
            self.theta = float(self.theta_valid)
            self.q = self.q_valid.copy()
            self.B = self.B_valid.copy()
            self.last_x = self.last_x_valid.copy()
            self.theta_dot = 0.0
            self.theta_ddot = 0.0
            self.dq[:] = 0.0
            self.ddq[:] = 0.0
            return

        # 8) Attrito + damping
        b = float(self.get_parameter('fric_visc').value)
        fc = float(self.get_parameter('fric_coul').value)
        eps = float(self.get_parameter('fric_eps').value)
        tau_fric = (
            b * self.theta_dot
            + fc * np.tanh(self.theta_dot / max(eps, 1e-9))
        )

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

        # 10) Diagnostica
        self.tau_fric_last = float(tau_fric)
        self.tau_damp_last = float(tau_damp)
        self.num_last = float(num)

        # 11) Integrazione Eulero esplicito
        self.theta_dot += self.theta_ddot * self.dt
        max_d = float(self.get_parameter('max_theta_dot').value)
        self.theta_dot = float(np.clip(self.theta_dot, -max_d, max_d))

        theta_next = self.theta + self.theta_dot * self.dt
        theta_next = self._clamp_theta_bounds(theta_next)

        if bool(self.get_parameter('limit_hold').value):
            if (abs(theta_next - self.theta) < 1e-12
                    and abs(self.theta_dot) > 1e-8):
                self.at_limit = True
                self.limit_dir = (
                    float(np.sign(self.theta_dot))
                    if abs(self.theta_dot) > 1e-9 else 1.0
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
        tau_rnea = pin.rnea(
            self.model, self.data, self.q, self.dq, self.ddq
        )
        self.tau_full = (tau_rnea - self.tau_ext - self.tau_pass).copy()

        # 14) Reazione equivalente su theta
        self.reaction_theta_last = float(
            self.proj_last + tau_fric + tau_damp
            - (self.tau_m + self.tau_pass_theta + self.tau_ext_theta)
        )

    # ============================================================
    # PUBLISH
    # ============================================================

    def publish(self) -> None:
        # --- /joint_states ---
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
            float(
                self.last_closure_norm
                if np.isfinite(self.last_closure_norm) else -1.0
            ),
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

        # --- /exo_dynamics/tau_ext_theta ---
        msg_tau = Float64()
        msg_tau.data = float(self.tau_ext_theta)
        self.pub_tau_ext_theta.publish(msg_tau)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExoDynamicsControlTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()