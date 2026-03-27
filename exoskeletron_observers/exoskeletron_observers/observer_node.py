#!/usr/bin/env python3
"""
observer_node.py
=================
Nodo ROS2 per la fault detection model-based sull'esoscheletro.

Implementa due observer complementari:

  OBSERVER 1 — Luenberger sullo stato (θ, θ̇)
  ─────────────────────────────────────────────
  Modello del plant ridotto a 1-DOF:

      M_eff · θ̈ = τ_m + τ_pass + τ_ext - proj - τ_fric_visc - τ_fric_coul - τ_damp

  L'observer predice θ̂ e θ̇_hat usando il modello completo (inclusi τ_ext
  misurato e attrito di Coulomb) e corregge con guadagni di Luenberger
  calcolati da pole-placement. Il residuo di stato (innovazione) è sensibile
  a guasti su sensori (encoder) e attuatori (coppia reale ≠ comandata).

  OBSERVER 2 — Inversione dell'ammittanza
  ─────────────────────────────────────────
  Ricostruisce τ_ext_hat dai riferimenti di traiettoria (θ_ref, θ̇_ref, θ̈_ref)
  invertendo il modello dell'admittance controller:

      τ̂_ext = M_v · θ̈_ref + D_v · θ̇_ref + K_v · (θ_ref - θ_eq)

  Il residuo τ_ext_meas - τ̂_ext è sensibile a guasti nel sensore di forza
  o nel loop di ammittanza. L'inversione viene invalidata (residuo = 0)
  quando il riferimento è saturato (clipping su θ_ref o θ̇_ref), perché in
  quel caso la relazione lineare non vale.

Topics
------
  Sub:  /joint_states                    sensor_msgs/JointState
        /exo_dynamics/tau_ext_theta      std_msgs/Float64
        /exo_dynamics/ff_terms           std_msgs/Float64MultiArray
        /torque_raw                      std_msgs/Float64
        /trajectory_ref                  std_msgs/Float64MultiArray
  Pub:  /observer/state_residual         std_msgs/Float64
        /observer/torque_residual        std_msgs/Float64
        /observer/state_rms              std_msgs/Float64
        /observer/torque_rms             std_msgs/Float64
        /observer/debug                  std_msgs/Float64MultiArray

Parametri ROS2
--------------
  joint_name             (str)    joint da osservare
  publish_rate           (float)  frequenza del loop [Hz]

  # Plant model
  fric_visc              (float)  attrito viscoso [Nm·s/rad]
  fric_coul              (float)  attrito Coulomb [Nm]
  fric_eps               (float)  soglia tanh Coulomb [rad/s]
  damping_theta          (float)  smorzamento addizionale [Nm·s/rad]

  # Observer 1 — Luenberger
  obs1_pole_1            (float)  polo 1 (negativo, stabile)
  obs1_pole_2            (float)  polo 2 (negativo, stabile)

  # Observer 2 — Inversione ammittanza
  # ATTENZIONE: questi parametri DEVONO corrispondere a quelli del nodo
  # admittance_controller (M_virt, D_virt, K_virt, theta_eq,
  # force_deadband, theta_ref_min, theta_ref_max, theta_dot_max).
  # Un disallineamento genera residui strutturali spuri.
  adm_M_virt             (float)  massa virtuale ammittanza
  adm_D_virt             (float)  smorzamento virtuale ammittanza
  adm_K_virt             (float)  rigidezza virtuale ammittanza
  adm_theta_eq           (float)  posizione di equilibrio
  adm_force_deadband     (float)  deadband forza dell'ammittanza
  adm_theta_ref_min      (float)  limite inferiore θ_ref
  adm_theta_ref_max      (float)  limite superiore θ_ref
  adm_theta_dot_max      (float)  limite velocità θ̇_ref

  # Filtri residui
  residual_filter_alpha  (float)  costante EMA
  rms_window             (int)    finestra RMS [campioni]
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


# ─────────────────────────────────────────────────────────────
# Utility
# ─────────────────────────────────────────────────────────────

def ema(prev: float, new: float, alpha: float) -> float:
    """Exponential moving average."""
    return alpha * new + (1.0 - alpha) * prev


def rms_from_buf(buf) -> float:
    """RMS da un buffer di valori al quadrato."""
    if len(buf) == 0:
        return 0.0
    return math.sqrt(sum(buf) / len(buf))


class ObserverNode(Node):

    def __init__(self):
        super().__init__('observer_node')

        # ── Parametri ─────────────────────────────────────────────
        self.declare_parameter('joint_name', 'rev_crank')
        self.declare_parameter('publish_rate', 200.0)

        # Plant model (devono corrispondere a dynamics_params.yaml)
        self.declare_parameter('fric_visc', 2.0)
        self.declare_parameter('fric_coul', 2.0)
        self.declare_parameter('fric_eps', 0.005)
        self.declare_parameter('damping_theta', 1.0)

        # Observer 1 — Luenberger poles
        self.declare_parameter('obs1_pole_1', -15.0)
        self.declare_parameter('obs1_pole_2', -20.0)

        # Observer 2 — Admittance inversion
        # NOTA: questi parametri DEVONO essere sincronizzati con quelli
        # dell'admittance_controller. Un disallineamento produce residui
        # strutturali spuri e falsi allarmi.
        self.declare_parameter('adm_M_virt', 0.5)
        self.declare_parameter('adm_D_virt', 5.0)
        self.declare_parameter('adm_K_virt', 2.0)
        self.declare_parameter('adm_theta_eq', 0.0)
        self.declare_parameter('adm_force_deadband', 0.0)
        self.declare_parameter('adm_theta_ref_min', -2.0)
        self.declare_parameter('adm_theta_ref_max', 2.0)
        self.declare_parameter('adm_theta_dot_max', 5.0)

        # Filtri residui
        self.declare_parameter('residual_filter_alpha', 0.05)
        self.declare_parameter('rms_window', 50)

        self._joint_name = self.get_parameter('joint_name').value
        rate = self.get_parameter('publish_rate').value
        self._dt = 1.0 / rate

        # ── Input ─────────────────────────────────────────────
        self._theta_meas = 0.0
        self._theta_dot_meas = 0.0
        self._tau_ext_meas = 0.0

        self._tau_m = 0.0
        self._M_eff = 1.0
        self._proj = 0.0
        self._tau_pass = 0.0

        # Trajectory ref
        self._theta_ref = 0.0
        self._theta_dot_ref = 0.0
        self._theta_ddot_ref = 0.0

        # Flags di ricezione
        self._js_received = False
        self._ff_received = False
        self._tau_ext_received = False
        self._tau_m_received = False
        self._traj_received = False

        # ── Observer 1 (Luenberger) ───────────────────────────
        self._theta_hat = 0.0
        self._theta_dot_hat = 0.0
        self._obs1_initialized = False

        # ── Observer 2 (Admittance Inversion) ─────────────────
        self._tau_ext_hat = 0.0
        self._obs2_valid = False  # False quando il riferimento è saturato

        # ── Residui ───────────────────────────────────────────
        self._state_res_filtered = 0.0
        self._torque_res_filtered = 0.0

        win = int(self.get_parameter('rms_window').value)
        self._state_res_buf = deque(maxlen=win)
        self._torque_res_buf = deque(maxlen=win)

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(Float64, '/exo_dynamics/tau_ext_theta', self._tau_ext_cb, 10)
        self.create_subscription(Float64MultiArray, '/exo_dynamics/ff_terms', self._ff_cb, 10)
        self.create_subscription(Float64, '/torque_raw', self._tau_m_cb, 10)
        self.create_subscription(Float64MultiArray, '/trajectory_ref', self._traj_cb, 10)

        # ── Publishers ────────────────────────────────────────
        self._pub_state = self.create_publisher(Float64, '/observer/state_residual', 10)
        self._pub_torque = self.create_publisher(Float64, '/observer/torque_residual', 10)
        self._pub_state_rms = self.create_publisher(Float64, '/observer/state_rms', 10)
        self._pub_torque_rms = self.create_publisher(Float64, '/observer/torque_rms', 10)
        self._pub_debug = self.create_publisher(Float64MultiArray, '/observer/debug', 10)

        self.create_timer(self._dt, self._update)

        self.get_logger().info(
            f"ObserverNode avviato | rate={rate:.0f} Hz | "
            f"poli=[{self.get_parameter('obs1_pole_1').value}, "
            f"{self.get_parameter('obs1_pole_2').value}]"
        )

    # ─────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────

    def _js_cb(self, msg):
        try:
            idx = msg.name.index(self._joint_name)
        except ValueError:
            return

        self._theta_meas = msg.position[idx]
        self._theta_dot_meas = msg.velocity[idx]
        self._js_received = True

    def _tau_ext_cb(self, msg):
        self._tau_ext_meas = msg.data
        self._tau_ext_received = True

    def _tau_m_cb(self, msg):
        self._tau_m = msg.data
        self._tau_m_received = True

    def _ff_cb(self, msg):
        d = msg.data
        if len(d) > 0:
            self._M_eff = max(d[0], 1e-6)
        if len(d) > 1:
            self._proj = d[1]
        if len(d) > 3:
            self._tau_pass = d[3]
        self._ff_received = True

    def _traj_cb(self, msg):
        d = msg.data
        if len(d) < 3:
            return
        self._theta_ref = d[0]
        self._theta_dot_ref = d[1]
        self._theta_ddot_ref = d[2]
        self._traj_received = True

    # ─────────────────────────────────────────────────────────
    # Loop principale
    # ─────────────────────────────────────────────────────────

    def _update(self):

        # FIX #4: aggiunto _tau_ext_received alla guardia
        if not (self._js_received and self._ff_received and
                self._tau_m_received and self._traj_received and
                self._tau_ext_received):
            return

        dt = self._dt

        # ── PARAMETRI PLANT ──────────────────────────────────
        fric_visc = self.get_parameter('fric_visc').value
        fric_coul = self.get_parameter('fric_coul').value
        fric_eps = self.get_parameter('fric_eps').value
        damping = self.get_parameter('damping_theta').value
        alpha = self.get_parameter('residual_filter_alpha').value

        # ─────────────────────────────────────────────────────
        # OBSERVER 1 — Luenberger
        # ─────────────────────────────────────────────────────
        #
        # Modello del plant ridotto:
        #   M_eff · θ̈ = τ_m + τ_pass + τ_ext - proj
        #               - (fric_visc + damping) · θ̇
        #               - fric_coul · tanh(θ̇ / fric_eps)
        #
        # L'observer usa il modello COMPLETO (inclusi τ_ext misurato
        # e attrito di Coulomb) per la predizione. Il Luenberger
        # corregge solo le discrepanze tra predizione e misura.
        # In questo modo il residuo è sensibile a guasti reali
        # (encoder, attuatore) e NON alla forza utente nominale.
        #
        # Guadagni L = [l1, l2] da pole-placement su:
        #   A = [[0, 1], [0, -D_lin/M]]   C = [1, 0]
        # dove D_lin = fric_visc + damping (parte linearizzabile).
        # Il Coulomb entra nel modello ma non nella linearizzazione
        # per il calcolo dei guadagni (è un termine non lineare).

        p1 = self.get_parameter('obs1_pole_1').value
        p2 = self.get_parameter('obs1_pole_2').value

        D_lin = fric_visc + damping
        D_over_M = D_lin / self._M_eff

        l1 = -(p1 + p2) - D_over_M
        l2 = p1 * p2 - l1 * D_over_M

        # Inizializzazione lazy dalla prima misura
        if not self._obs1_initialized:
            self._theta_hat = self._theta_meas
            self._theta_dot_hat = self._theta_dot_meas
            self._obs1_initialized = True

        # FIX #3: attrito completo (viscoso + Coulomb + smorzamento)
        tau_fric_visc = fric_visc * self._theta_dot_hat
        tau_fric_coul = fric_coul * math.tanh(self._theta_dot_hat / fric_eps)
        tau_damp = damping * self._theta_dot_hat
        tau_dissipative = tau_fric_visc + tau_fric_coul + tau_damp

        # FIX #2: incluso τ_ext_meas nel modello dell'observer
        theta_ddot_model = (
            self._tau_m
            + self._tau_pass
            + self._tau_ext_meas        # ← era mancante
            - self._proj
            - tau_dissipative
        ) / self._M_eff

        # Innovazione (errore di predizione)
        innov = self._theta_meas - self._theta_hat
        state_res = innov

        # FIX #1: Eulero esplicito corretto — entrambi gli stati
        # aggiornati sullo stato VECCHIO (non semi-implicito)
        theta_dot_hat_new = (
            self._theta_dot_hat + dt * (theta_ddot_model + l2 * innov)
        )
        theta_hat_new = (
            self._theta_hat + dt * (self._theta_dot_hat + l1 * innov)
        )
        self._theta_dot_hat = theta_dot_hat_new
        self._theta_hat = theta_hat_new

        # ─────────────────────────────────────────────────────
        # OBSERVER 2 — Inversione Ammittanza
        # ─────────────────────────────────────────────────────
        #
        # Il modello dell'admittance controller è:
        #   M_v · θ̈_v + D_v · θ̇_v + K_v · (θ_v - θ_eq) = τ_ext
        #
        # Invertendo:
        #   τ̂_ext = M_v · θ̈_ref + D_v · θ̇_ref + K_v · (θ_ref - θ_eq)
        #
        # ATTENZIONE: l'inversione è valida SOLO quando il
        # riferimento non è saturato. Se θ_ref è clippato ai
        # limiti, o θ̇_ref è clippato, o la forza è dentro la
        # deadband, la relazione lineare non vale e il residuo
        # sarebbe spurio. In questi casi il residuo viene azzerato
        # e flaggato come non valido.

        Mv = self.get_parameter('adm_M_virt').value
        Dv = self.get_parameter('adm_D_virt').value
        Kv = self.get_parameter('adm_K_virt').value
        theta_eq = self.get_parameter('adm_theta_eq').value
        deadband = self.get_parameter('adm_force_deadband').value
        th_min = self.get_parameter('adm_theta_ref_min').value
        th_max = self.get_parameter('adm_theta_ref_max').value
        dv_max = self.get_parameter('adm_theta_dot_max').value

        # FIX #6: verifica se il riferimento è in zona di saturazione
        sat_margin = 1e-3  # margine per considerare "a saturazione"
        ref_pos_saturated = (
            self._theta_ref <= th_min + sat_margin or
            self._theta_ref >= th_max - sat_margin
        )
        ref_vel_saturated = (
            abs(self._theta_dot_ref) >= dv_max - sat_margin
        )
        in_deadband = abs(self._tau_ext_meas) < deadband

        self._obs2_valid = not (ref_pos_saturated or ref_vel_saturated or in_deadband)

        if self._obs2_valid:
            self._tau_ext_hat = (
                Mv * self._theta_ddot_ref +
                Dv * self._theta_dot_ref +
                Kv * (self._theta_ref - theta_eq)
            )
            torque_res = self._tau_ext_meas - self._tau_ext_hat
        else:
            # Riferimento saturato: inversione non valida, residuo azzerato
            self._tau_ext_hat = float('nan')
            torque_res = 0.0

        # ── FILTRI ───────────────────────────────────────────
        self._state_res_filtered = ema(self._state_res_filtered, state_res, alpha)
        self._torque_res_filtered = ema(self._torque_res_filtered, torque_res, alpha)

        self._state_res_buf.append(state_res ** 2)
        self._torque_res_buf.append(torque_res ** 2)

        # FIX #7: calcolo e pubblicazione RMS
        state_rms = rms_from_buf(self._state_res_buf)
        torque_rms = rms_from_buf(self._torque_res_buf)

        # ── PUBBLICAZIONE ────────────────────────────────────

        msg = Float64()
        msg.data = state_res
        self._pub_state.publish(msg)

        msg = Float64()
        msg.data = torque_res
        self._pub_torque.publish(msg)

        msg = Float64()
        msg.data = state_rms
        self._pub_state_rms.publish(msg)

        msg = Float64()
        msg.data = torque_rms
        self._pub_torque_rms.publish(msg)

        # Debug: layout esteso
        # [0]  theta_hat
        # [1]  theta_dot_hat
        # [2]  state_residual (innovazione)
        # [3]  tau_ext_hat (NaN se obs2 non valido)
        # [4]  torque_residual
        # [5]  state_res_filtered (EMA)
        # [6]  torque_res_filtered (EMA)
        # [7]  state_rms
        # [8]  torque_rms
        # [9]  theta_meas
        # [10] theta_dot_meas
        # [11] tau_m
        # [12] tau_ext_meas
        # [13] theta_ref
        # [14] theta_dot_ref
        # [15] theta_ddot_ref
        # [16] obs2_valid (1.0 / 0.0)
        dbg = Float64MultiArray()
        dbg.data = [
            self._theta_hat,                                    # 0
            self._theta_dot_hat,                                # 1
            state_res,                                          # 2
            self._tau_ext_hat if self._obs2_valid else 0.0,     # 3
            torque_res,                                         # 4
            self._state_res_filtered,                           # 5
            self._torque_res_filtered,                          # 6
            state_rms,                                          # 7
            torque_rms,                                         # 8
            self._theta_meas,                                   # 9
            self._theta_dot_meas,                               # 10
            self._tau_m,                                        # 11
            self._tau_ext_meas,                                 # 12
            self._theta_ref,                                    # 13
            self._theta_dot_ref,                                # 14
            self._theta_ddot_ref,                               # 15
            1.0 if self._obs2_valid else 0.0,                   # 16
        ]
        self._pub_debug.publish(dbg)


def main():
    rclpy.init()
    node = ObserverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()