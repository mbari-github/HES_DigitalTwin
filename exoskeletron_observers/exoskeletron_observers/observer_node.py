#!/usr/bin/env python3
"""
observer_node.py
=================
Nodo ROS2 per la fault detection model-based sull'esoscheletro.

Implementa quattro strati di detection complementari:

  OBSERVER 1 — Luenberger sullo stato (θ, θ̇)
  ─────────────────────────────────────────────
  Modello del plant ridotto a 1-DOF. Predice θ̂ e θ̇_hat usando il modello
  completo e corregge con guadagni di Luenberger. Il residuo (innovazione)
  è sensibile a guasti encoder e attuatore, ma ha detection latency di
  ~1-1.5 s per fault su τ_ext (canale 0) perché la risposta è mediata
  dalla catena ammittanza → tracking.

  OBSERVER 2 — Inversione dell'ammittanza
  ─────────────────────────────────────────
  Ricostruisce τ̂_ext dai riferimenti di traiettoria invertendo il modello
  dell'admittance controller. Sensibile a fault nel loop di ammittanza.
  Invalidato quando il riferimento è saturato.

  OBSERVER 3 — Momento generalizzato 
  ─────────────────────────────────────────────
  Stima diretta della discrepanza sulla coppia esterna, senza bisogno di
  calcolare θ̈ (derivata rumorosa). Lavora sull'integrale del momento:

      p(t) = M_eff · θ̇

      σ̇ = τ_known + r_mom          σ(0) = p(0)
      r_mom = K_mom · (p - σ)

  dove τ_known = τ_m + τ_pass + τ_ext_meas - proj - friction.
  In regime stazionario: r_mom → (τ_ext_reale - τ_ext_misurato).

  Con K_mom = 50 la costante di tempo è ~20 ms: detection latency
  ~40-60 ms per un offset di 2 Nm (vs ~1.5 s del Luenberger).

  GUARDIA 4 — Derivata di τ_ext  ★ NUOVO
  ─────────────────────────────────────────
  Monitora |Δτ_ext / Δt|. Una forza umana fisica non può produrre
  gradini istantanei (inerzia braccio, compliance tessuti, banda
  muscolare ≈ 5-10 Hz). Un salto di 2 Nm in 5 ms = 400 Nm/s è
  necessariamente artificiale. Detection: 1 campione (~5 ms).
  Cieco ai drift lenti.

Topics
------
  Sub:  /joint_states                    sensor_msgs/JointState
        /exo_dynamics/tau_ext_theta      std_msgs/Float64
        /exo_dynamics/ff_terms           std_msgs/Float64MultiArray
        /torque_raw                      std_msgs/Float64
        /trajectory_ref                  std_msgs/Float64MultiArray
  Pub:  /observer/state_residual         std_msgs/Float64
        /observer/torque_residual        std_msgs/Float64
        /observer/momentum_residual      std_msgs/Float64     
        /observer/tau_ext_rate_alarm     std_msgs/Float64     
        /observer/state_rms              std_msgs/Float64
        /observer/torque_rms             std_msgs/Float64
        /observer/debug                  std_msgs/Float64MultiArray  (21 campi)

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
  adm_M_virt             (float)  massa virtuale ammittanza
  adm_D_virt             (float)  smorzamento virtuale ammittanza
  adm_K_virt             (float)  rigidezza virtuale ammittanza
  adm_theta_eq           (float)  posizione di equilibrio
  adm_force_deadband     (float)  deadband forza dell'ammittanza
  adm_theta_ref_min      (float)  limite inferiore θ_ref
  adm_theta_ref_max      (float)  limite superiore θ_ref
  adm_theta_dot_max      (float)  limite velocità θ̇_ref

  # Observer 3 — Momento generalizzato
  mom_obs_gain           (float)  guadagno K_mom [1/s] (default 50 → τ ≈ 20 ms)

  # Guardia 4 — Derivata τ_ext
  tau_ext_max_rate       (float)  soglia |dτ_ext/dt| [Nm/s] (default 100)

  # Filtri residui
  residual_filter_alpha  (float)  costante EMA
  rms_window             (int)    finestra RMS [campioni]
  torque_res_max         (float)  clamp residuo coppia [Nm]
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
        self.declare_parameter('adm_M_virt', 0.5)
        self.declare_parameter('adm_D_virt', 5.0)
        self.declare_parameter('adm_K_virt', 2.0)
        self.declare_parameter('adm_theta_eq', 0.0)
        self.declare_parameter('adm_force_deadband', 0.0)
        self.declare_parameter('adm_theta_ref_min', -2.0)
        self.declare_parameter('adm_theta_ref_max', 2.0)
        self.declare_parameter('adm_theta_dot_max', 5.0)

        # Observer 3 — Momentum observer
        # K_mom [1/s]: guadagno del momentum observer.
        # Costante di tempo τ ≈ 1/K_mom.
        #   K_mom = 50  → τ ≈ 20 ms  (detection rapida)
        #   K_mom = 20  → τ ≈ 50 ms  (più filtrato)
        #   K_mom = 100 → τ ≈ 10 ms  (molto rapido, più sensibile al rumore)
        self.declare_parameter('mom_obs_gain', 50.0)

        # Guardia 4 — Rate-of-change τ_ext
        # Soglia in Nm/s. Riferimenti fisiologici:
        #   - Rate massimo forza umana: ~5-20 Nm/s
        #   - Fault offset 2 Nm in 5 ms → 400 Nm/s
        # 100 Nm/s offre ampio margine per l'uso normale.
        self.declare_parameter('tau_ext_max_rate', 100.0)

        # Filtri residui
        self.declare_parameter('residual_filter_alpha', 0.05)
        self.declare_parameter('rms_window', 50)
        self.declare_parameter('torque_res_max', 0.5)

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
        self._obs2_valid = False

        # ── Observer 3 (Momentum) ─────────────────────────────
        self._mom_sigma = 0.0       # integratore σ(t)
        self._r_momentum = 0.0      # residuo r(t)
        self._M_eff_prev = 1.0      # M_eff al passo precedente
        self._mom_initialized = False

        # ── Guardia 4 (Derivata τ_ext) ────────────────────────
        self._tau_ext_prev = 0.0
        self._tau_ext_rate = 0.0
        self._rate_alarm = False
        self._rate_guard_initialized = False

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
        self._pub_momentum = self.create_publisher(Float64, '/observer/momentum_residual', 10)
        self._pub_rate_alarm = self.create_publisher(Float64, '/observer/tau_ext_rate_alarm', 10)
        self._pub_state_rms = self.create_publisher(Float64, '/observer/state_rms', 10)
        self._pub_torque_rms = self.create_publisher(Float64, '/observer/torque_rms', 10)
        self._pub_debug = self.create_publisher(Float64MultiArray, '/observer/debug', 10)

        self.create_timer(self._dt, self._update)

        self.get_logger().info(
            f"ObserverNode avviato | rate={rate:.0f} Hz | "
            f"poli=[{self.get_parameter('obs1_pole_1').value}, "
            f"{self.get_parameter('obs1_pole_2').value}] | "
            f"K_mom={self.get_parameter('mom_obs_gain').value} | "
            f"rate_max={self.get_parameter('tau_ext_max_rate').value} Nm/s"
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

        p1 = self.get_parameter('obs1_pole_1').value
        p2 = self.get_parameter('obs1_pole_2').value

        D_lin = fric_visc + damping
        D_over_M = D_lin / self._M_eff

        l1 = -(p1 + p2) - D_over_M
        l2 = p1 * p2 - l1 * D_over_M

        if not self._obs1_initialized:
            self._theta_hat = self._theta_meas
            self._theta_dot_hat = self._theta_dot_meas
            self._obs1_initialized = True

        # Attrito completo su θ̇_hat (per il modello del Luenberger)
        tau_fric_visc = fric_visc * self._theta_dot_hat
        tau_fric_coul = fric_coul * math.tanh(self._theta_dot_hat / fric_eps)
        tau_damp = damping * self._theta_dot_hat
        tau_dissipative = tau_fric_visc + tau_fric_coul + tau_damp

        theta_ddot_model = (
            self._tau_m
            + self._tau_pass
            + self._tau_ext_meas
            - self._proj
            - tau_dissipative
        ) / self._M_eff

        innov = self._theta_meas - self._theta_hat
        state_res = innov

        # Eulero esplicito corretto
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

        Mv = self.get_parameter('adm_M_virt').value
        Dv = self.get_parameter('adm_D_virt').value
        Kv = self.get_parameter('adm_K_virt').value
        theta_eq = self.get_parameter('adm_theta_eq').value
        deadband = self.get_parameter('adm_force_deadband').value
        th_min = self.get_parameter('adm_theta_ref_min').value
        th_max = self.get_parameter('adm_theta_ref_max').value
        dv_max = self.get_parameter('adm_theta_dot_max').value

        sat_margin = 1e-3
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
            self._tau_ext_hat = float('nan')
            torque_res = 0.0

        res_max = self.get_parameter('torque_res_max').value
        torque_res = max(-res_max, min(res_max, torque_res))

        # ─────────────────────────────────────────────────────
        # OBSERVER 3 — Momento Generalizzato
        # ─────────────────────────────────────────────────────
        #
        # Principio: p = M_eff · θ̇  (momento generalizzato)
        #
        #   dp/dt = Ṁ_eff·θ̇ + τ_m + τ_pass + τ_ext_real − proj − friction
        #
        # L'observer integra i termini noti e confronta con p:
        #
        #   σ̇ = τ_known + r_mom         σ(0) = p(0)
        #   r_mom = K_mom · (p − σ)
        #
        # In regime:  r_mom → τ_ext_real − τ_ext_meas
        #
        # Il termine Ṁ_eff·θ̇ (variazione inerzia con la configurazione)
        # viene compensato numericamente:
        #   Δp_M ≈ (M_eff_k − M_eff_{k-1}) · θ̇

        K_mom = self.get_parameter('mom_obs_gain').value

        p_now = self._M_eff * self._theta_dot_meas

        if not self._mom_initialized:
            self._mom_sigma = p_now
            self._M_eff_prev = self._M_eff
            self._mom_initialized = True

        # Attrito calcolato su θ̇ MISURATO (coerente con p = M·θ̇_meas)
        tau_fric_visc_m = fric_visc * self._theta_dot_meas
        tau_fric_coul_m = fric_coul * math.tanh(self._theta_dot_meas / fric_eps)
        tau_damp_m = damping * self._theta_dot_meas
        tau_dissipative_m = tau_fric_visc_m + tau_fric_coul_m + tau_damp_m

        # Termini noti di dp/dt
        tau_known = (
            self._tau_m
            + self._tau_pass
            + self._tau_ext_meas
            - self._proj
            - tau_dissipative_m
        )

        # Compensazione variazione inerzia: ∫ Ṁ·θ̇ dt ≈ ΔM · θ̇
        delta_p_M = (self._M_eff - self._M_eff_prev) * self._theta_dot_meas

        # Aggiornamento integratore
        self._mom_sigma += (tau_known + self._r_momentum) * dt + delta_p_M

        # Residuo
        self._r_momentum = K_mom * (p_now - self._mom_sigma)

        self._M_eff_prev = self._M_eff

        # ─────────────────────────────────────────────────────
        # GUARDIA 4 — Derivata τ_ext
        # ─────────────────────────────────────────────────────

        max_rate = self.get_parameter('tau_ext_max_rate').value

        if not self._rate_guard_initialized:
            self._tau_ext_prev = self._tau_ext_meas
            self._rate_guard_initialized = True

        self._tau_ext_rate = abs(self._tau_ext_meas - self._tau_ext_prev) / dt
        self._rate_alarm = self._tau_ext_rate > max_rate
        self._tau_ext_prev = self._tau_ext_meas

        # ── FILTRI ───────────────────────────────────────────
        self._state_res_filtered = ema(self._state_res_filtered, state_res, alpha)
        self._torque_res_filtered = ema(self._torque_res_filtered, torque_res, alpha)

        self._state_res_buf.append(state_res ** 2)
        self._torque_res_buf.append(torque_res ** 2)

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
        msg.data = self._r_momentum
        self._pub_momentum.publish(msg)

        msg = Float64()
        msg.data = 1.0 if self._rate_alarm else 0.0
        self._pub_rate_alarm.publish(msg)

        msg = Float64()
        msg.data = state_rms
        self._pub_state_rms.publish(msg)

        msg = Float64()
        msg.data = torque_rms
        self._pub_torque_rms.publish(msg)

        # Debug: layout esteso (21 campi)
        # [0]  theta_hat
        # [1]  theta_dot_hat
        # [2]  state_residual (innovazione)
        # [3]  tau_ext_hat (0 se obs2 non valido)
        # [4]  torque_residual (obs2, clampato)
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
        # [17] r_momentum (obs3)         
        # [18] tau_ext_rate [Nm/s]       
        # [19] rate_alarm (1.0 / 0.0)    
        # [20] p_momentum (M_eff · θ̇)   
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
            self._r_momentum,                                   # 17
            self._tau_ext_rate,                                 # 18
            1.0 if self._rate_alarm else 0.0,                   # 19
            p_now,                                              # 20
        ]
        self._pub_debug.publish(dbg)


def main():
    rclpy.init()
    node = ObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
