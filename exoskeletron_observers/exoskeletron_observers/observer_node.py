#!/usr/bin/env python3
"""
observer_node.py
================
Nodo ROS2 con due osservatori model-based per la fault detection sul digital
twin dell'esoscheletro.

Motivazione
-----------
Il FaultMonitorMode della state machine esegue soglie semplici su tau e velocità.
Questi osservatori aggiungono un secondo livello di rilevamento basato sul
**residuo** tra il valore misurato e quello stimato dal modello: un residuo
persistentemente non nullo indica un guasto, anche prima che venga superata
una soglia assoluta.

Equazione del plant (scalare ridotta, DOF: rev_crank = theta):

    M_eff * θ̈ = τ_m + τ_pass - proj - τ_fric - τ_damp + τ_ext

dove:
    τ_fric = fric_visc * θ̇ + fric_coul * tanh(θ̇ / fric_eps)  [non lineare]
    τ_damp = damping_theta * θ̇                                  [lineare]
    M_eff, proj, τ_pass  → pubblicati da /exo_dynamics/ff_terms
    τ_m                  → coppia comandata dal trajectory controller
    τ_ext                → forza utente proiettata su theta

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
OBSERVER 1 — State Observer di Luenberger  (fault detection canale 3: encoder)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Stima lo stato [θ̂, θ̇̂] usando il modello del plant e la coppia τ_m (sempre
affidabile), senza fidarsi dell'encoder. Il residuo r = θ_meas - θ̂ è nullo
in nominale e cresce in presenza di guasti sull'encoder (canale 3).

Il modello NON include τ_ext nel calcolo di θ̈: questo è intenzionale.
L'observer 1 è progettato per rilevare fault sull'encoder, non sulla forza.
Se includessimo τ_ext (potenzialmente faulted su canale 0), avveleneremmo
la predizione. Senza τ_ext, il Luenberger ha un residuo nominale non-zero
durante il moto guidato dall'utente, ma i guadagni di correzione (l1, l2)
lo mantengono limitato. Un fault sull'encoder produce un salto netto che
supera questo baseline.

Modello lineare approssimato  x = [θ, θ̇],  y = θ:

    A = [[0,         1       ],      C = [1, 0]
         [0,  -D/M_eff       ]]

Gain via pole placement (poli p1, p2 reali negativi):

    l1 = -(p1+p2) - D/M_eff
    l2 =  p1·p2 - l1·(D/M_eff)

Aggiornamento Eulero esplicito: usa θ̇̂[k] (vecchio) per calcolare θ̂[k+1].

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
OBSERVER 2 — Force Balance  (fault detection canale 0: sensore forza)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Stima τ_ext dal bilancio algebrico di forze — funziona anche a sistema FERMO
(a differenza del momentum observer che richiede θ̇ ≠ 0).

Dal bilancio:
    τ_ext = M_eff·θ̈ + proj + τ_fric_lin - τ_m - τ_pass

dove τ_fric_lin = (fric_visc + damping_theta)·θ̇  (solo parte lineare — il
termine di Coulomb viene omesso deliberatamente: a θ̇≈0 il suo segno è ambiguo
e aggiungerebbe ±fric_coul Nm di bias al residuo nominale).

θ̈ stimato con dirty-derivative filtrato (EMA su (θ̇[k]-θ̇[k-1])/dt):
    θ̈ˆ[k] = α_acc·(θ̇[k]-θ̇[k-1])/dt + (1-α_acc)·θ̈ˆ[k-1]

τ̂_ext ulteriormente filtrato con EMA:
    τ̂_ext[k] = α_tau·τ_ext_inst[k] + (1-α_tau)·τ̂_ext[k-1]

Residuo:  r_torque = τ_ext_meas - τ̂_ext

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Topics
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Sottoscrizioni:
  /joint_states                   sensor_msgs/JointState     [può essere faulted canale 3]
  /exo_dynamics/tau_ext_theta     std_msgs/Float64           [può essere faulted canale 0]
  /exo_dynamics/ff_terms          std_msgs/Float64MultiArray [sempre pulito]
  /torque_raw                     std_msgs/Float64           [coppia pulita, remap da launch]
  /exo_bridge/status              std_msgs/Float64MultiArray [θ̇ sempre affidabile]

Pubblicazioni:
  /observer/state_residual    std_msgs/Float64            r_state  = θ_meas - θ̂
  /observer/torque_residual   std_msgs/Float64            r_torque = τ_ext_meas - τ̂_ext
  /observer/debug             std_msgs/Float64MultiArray  diagnostica completa (15 campi)

Layout /observer/debug:
  [0]  theta_hat             stima posizione  (observer 1)
  [1]  theta_dot_hat         stima velocità   (observer 1)
  [2]  state_residual        r_state grezzo
  [3]  tau_ext_hat           stima forza esterna (observer 2)
  [4]  torque_residual       r_torque grezzo
  [5]  state_res_filtered    r_state filtrato EMA
  [6]  torque_res_filtered   r_torque filtrato EMA
  [7]  theta_meas            θ da joint_states (possibilmente faulted)
  [8]  theta_dot_meas        θ̇ da joint_states (possibilmente faulted)
  [9]  tau_m                 coppia comandata (da /torque_raw)
  [10] tau_ext_meas          τ_ext da sensore (possibilmente faulted)
  [11] M_eff                 inerzia effettiva corrente
  [12] proj                  B^T·(M·Ḃ·θ̇ + h)
  [13] theta_ddot_hat        θ̈ stimata (dirty derivative filtrato)
  [14] tau_pass              coppie passive proiettate

Parametri ROS2
━━━━━━━━━━━━━━
  joint_name              (str,   default 'rev_crank')
  publish_rate            (float, default 200.0)  [Hz]

  # Modello plant (devono corrispondere a dynamics_params.yaml)
  fric_visc               (float, default 2.0)    [Nm·s/rad]
  fric_coul               (float, default 2.0)    [Nm]
  fric_eps                (float, default 0.005)  [rad/s]
  damping_theta           (float, default 1.0)    [Nm·s/rad]

  # Observer 1 — Luenberger
  obs1_pole_1             (float, default -15.0)  polo 1 (reale negativo)
  obs1_pole_2             (float, default -20.0)  polo 2 (reale negativo)

  # Observer 2 — Force Balance
  obs2_alpha_acc          (float, default 0.02)   EMA sul dirty-derivative θ̈
  obs2_alpha_tau          (float, default 0.05)   EMA su τ̂_ext

  # Filtro EMA sui residui pubblicati
  residual_filter_alpha   (float, default 0.05)   coefficiente EMA

  # Finestra buffer RMS (campioni)
  rms_window              (int,   default 50)
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


# ─────────────────────────────────────────────────────────────────────────────
# Filtro EMA helper
# ─────────────────────────────────────────────────────────────────────────────

def ema(prev: float, new: float, alpha: float) -> float:
    """Exponential Moving Average. alpha vicino a 1 → poco filtraggio."""
    return alpha * new + (1.0 - alpha) * prev


# ─────────────────────────────────────────────────────────────────────────────
# Nodo principale
# ─────────────────────────────────────────────────────────────────────────────

class ObserverNode(Node):

    def __init__(self):
        super().__init__('observer_node')

        # ── Parametri ───────────────────────────────────────────────────────
        self.declare_parameter('joint_name',            'rev_crank')
        self.declare_parameter('publish_rate',          200.0)

        # Modello plant
        self.declare_parameter('fric_visc',             2.0)
        self.declare_parameter('fric_coul',             2.0)
        self.declare_parameter('fric_eps',              0.005)
        self.declare_parameter('damping_theta',         1.0)

        # Observer 1 — Luenberger
        self.declare_parameter('obs1_pole_1',          -15.0)
        self.declare_parameter('obs1_pole_2',          -20.0)

        # Observer 2 — Force Balance
        self.declare_parameter('obs2_alpha_acc',        0.02)
        self.declare_parameter('obs2_alpha_tau',        0.05)

        # Filtro residui
        self.declare_parameter('residual_filter_alpha', 0.05)
        self.declare_parameter('rms_window',            50)

        # ── Lettura parametri fissi ──────────────────────────────────────────
        self._joint_name    = str(self.get_parameter('joint_name').value)
        publish_rate        = float(self.get_parameter('publish_rate').value)
        self._dt            = 1.0 / publish_rate

        # ── Stato interno — segnali in ingresso ─────────────────────────────
        self._theta_meas        = 0.0
        self._theta_dot_meas    = 0.0
        self._tau_ext_meas      = 0.0

        self._tau_m             = 0.0
        self._M_eff             = 1.0
        self._proj              = 0.0
        self._g_proj            = 0.0
        self._tau_pass          = 0.0

        self._theta_bridge      = 0.0
        self._theta_dot_bridge  = 0.0
        self._bridge_received   = False

        self._js_received       = False
        self._ff_received       = False
        self._tau_ext_received  = False
        self._tau_m_received    = False

        # ── Stato Observer 1 — Luenberger ───────────────────────────────────
        self._theta_hat         = 0.0
        self._theta_dot_hat     = 0.0
        self._obs1_initialized  = False

        # ── Stato Observer 2 — Force Balance ────────────────────────────────
        self._tau_ext_hat       = 0.0
        self._theta_dot_prev    = 0.0
        self._theta_ddot_hat    = 0.0
        self._obs2_initialized  = False

        # ── Residui filtrati ─────────────────────────────────────────────────
        self._state_res_filtered   = 0.0
        self._torque_res_filtered  = 0.0

        win = int(self.get_parameter('rms_window').value)
        self._state_res_buf   = deque(maxlen=win)
        self._torque_res_buf  = deque(maxlen=win)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(
            Float64, '/exo_dynamics/tau_ext_theta', self._tau_ext_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/exo_dynamics/ff_terms', self._ff_terms_cb, 10)
        self.create_subscription(
            Float64, '/torque_raw', self._torque_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/exo_bridge/status', self._bridge_status_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_state_res  = self.create_publisher(
            Float64, '/observer/state_residual', 10)
        self._pub_torque_res = self.create_publisher(
            Float64, '/observer/torque_residual', 10)
        self._pub_debug      = self.create_publisher(
            Float64MultiArray, '/observer/debug', 10)

        # ── Timer principale ─────────────────────────────────────────────────
        self.create_timer(self._dt, self._update)

        self.get_logger().info(
            f'ObserverNode avviato | joint={self._joint_name} '
            f'rate={publish_rate:.0f}Hz dt={self._dt*1e3:.2f}ms'
        )

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _js_cb(self, msg: JointState) -> None:
        try:
            idx = msg.name.index(self._joint_name)
        except ValueError:
            return
        if idx < len(msg.position):
            self._theta_meas = float(msg.position[idx])
        if idx < len(msg.velocity):
            self._theta_dot_meas = float(msg.velocity[idx])
        self._js_received = True

    def _tau_ext_cb(self, msg: Float64) -> None:
        self._tau_ext_meas = float(msg.data)
        self._tau_ext_received = True

    def _ff_terms_cb(self, msg: Float64MultiArray) -> None:
        d = msg.data
        if len(d) > 0:
            m = float(d[0])
            self._M_eff = m if m > 1e-6 else 1.0
        if len(d) > 1:
            self._proj = float(d[1])
        if len(d) > 2:
            self._g_proj = float(d[2])
        if len(d) > 3:
            self._tau_pass = float(d[3])
        self._ff_received = True

    def _torque_cb(self, msg: Float64) -> None:
        self._tau_m = float(msg.data)
        self._tau_m_received = True

    def _bridge_status_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 4:
            return
        self._theta_bridge      = float(msg.data[2])
        self._theta_dot_bridge  = float(msg.data[3])
        self._bridge_received   = True

    # =========================================================================
    # Helpers numerici
    # =========================================================================

    @staticmethod
    def _safe_sq(x: float, limit: float = 1e6) -> float:
        x_clamped = max(-limit, min(limit, x))
        return x_clamped * x_clamped

    @staticmethod
    def _is_valid(x: float) -> bool:
        return math.isfinite(x)

    def _reset_observer1(self, reason: str = '') -> None:
        self._theta_hat         = self._theta_meas
        self._theta_dot_hat     = self._theta_dot_meas
        self._obs1_initialized  = False
        if reason:
            self.get_logger().warn(f'Observer 1 reset: {reason}')

    def _reset_observer2(self, reason: str = '') -> None:
        self._tau_ext_hat       = 0.0
        self._theta_dot_prev    = self._theta_dot_meas
        self._theta_ddot_hat    = 0.0
        self._obs2_initialized  = False
        if reason:
            self.get_logger().warn(f'Observer 2 reset: {reason}')

    # =========================================================================
    # Loop principale
    # =========================================================================

    def _update(self) -> None:

        if not (self._js_received and self._ff_received and self._tau_m_received):
            return

        dt = self._dt

        # ── Lettura parametri (aggiornabili a runtime) ───────────────────────
        fric_visc     = float(self.get_parameter('fric_visc').value)
        fric_coul     = float(self.get_parameter('fric_coul').value)
        fric_eps      = float(self.get_parameter('fric_eps').value)
        damping_theta = float(self.get_parameter('damping_theta').value)
        alpha         = float(self.get_parameter('residual_filter_alpha').value)

        M_eff    = self._M_eff
        proj     = self._proj
        tau_pass = self._tau_pass
        tau_m    = self._tau_m

        # Saturazione fisica (difesa contro NaN/Inf)
        TAU_MAX   = 50.0
        VEL_MAX   = 20.0
        THETA_MAX = 5.0

        tau_m    = max(-TAU_MAX,  min(TAU_MAX,  tau_m))
        proj     = max(-TAU_MAX,  min(TAU_MAX,  proj))
        tau_pass = max(-TAU_MAX,  min(TAU_MAX,  tau_pass))

        # ── Attrito (modello del plant) ─────────────────────────────────────
        def tau_friction(theta_dot: float) -> float:
            return (fric_visc * theta_dot
                    + fric_coul * math.tanh(theta_dot / max(fric_eps, 1e-9)))

        def tau_damping(theta_dot: float) -> float:
            return damping_theta * theta_dot

        # ─────────────────────────────────────────────────────────────────────
        # OBSERVER 1 — Luenberger
        # ─────────────────────────────────────────────────────────────────────
        #
        # Modello SENZA τ_ext (intenzionale):
        #   θ̈_model = (τ_m + τ_pass - proj - τ_fric - τ_damp) / M_eff
        #
        # Perché senza τ_ext: questo observer rileva fault sull'encoder
        # (canale 3). τ_ext potrebbe essere corrotto (canale 0) e
        # includerlo avvelenerebbe la predizione. Il residuo nominale
        # non-zero durante il moto è gestito dai guadagni di Luenberger
        # che lo mantengono limitato. Un fault sull'encoder produce un
        # salto netto molto più grande di questo baseline.

        p1 = float(self.get_parameter('obs1_pole_1').value)
        p2 = float(self.get_parameter('obs1_pole_2').value)

        D_lin    = fric_visc + damping_theta
        D_over_M = D_lin / M_eff
        l1 = -(p1 + p2) - D_over_M
        l2 = p1 * p2 - l1 * D_over_M

        if not self._obs1_initialized:
            self._theta_hat     = self._theta_meas
            self._theta_dot_hat = self._theta_dot_meas
            self._obs1_initialized = True
            self.get_logger().info(
                f'Observer 1 inizializzato | theta={self._theta_hat:.4f} '
                f'l1={l1:.2f} l2={l2:.2f} (poli: {p1}, {p2})'
            )

        tau_fric_hat = tau_friction(self._theta_dot_hat)
        tau_damp_hat = tau_damping(self._theta_dot_hat)

        theta_ddot_model = (
            tau_m + tau_pass - proj - tau_fric_hat - tau_damp_hat
        ) / M_eff

        # Innovazione PRIMA dell'aggiornamento
        innov = self._theta_meas - self._theta_hat
        state_residual = innov

        # Eulero esplicito: entrambi calcolati sullo stato VECCHIO
        new_theta_dot = self._theta_dot_hat + dt * (theta_ddot_model + l2 * innov)
        new_theta     = self._theta_hat     + dt * (self._theta_dot_hat + l1 * innov)

        # Clamp fisico
        self._theta_dot_hat = max(-VEL_MAX,   min(VEL_MAX,   new_theta_dot))
        self._theta_hat     = max(-THETA_MAX, min(THETA_MAX, new_theta))

        # Guard NaN/Inf
        if not (self._is_valid(self._theta_hat) and
                self._is_valid(self._theta_dot_hat)):
            self._reset_observer1('stato non finito rilevato')
            state_residual = 0.0

        # ─────────────────────────────────────────────────────────────────────
        # OBSERVER 2 — Force Balance
        # ─────────────────────────────────────────────────────────────────────
        #
        # τ_ext = M_eff·θ̈ + proj + τ_fric_lin - τ_m - τ_pass
        #
        # Solo parte lineare dell'attrito (viscoso + damping).
        # Il Coulomb è omesso: a θ̇≈0 il segno è ambiguo e aggiungerebbe
        # ±fric_coul Nm di bias al residuo nominale.

        if self._bridge_received:
            theta_dot_fb = self._theta_dot_bridge
        else:
            theta_dot_fb = self._theta_dot_meas
        theta_dot_fb = max(-VEL_MAX, min(VEL_MAX, theta_dot_fb))

        if not self._obs2_initialized:
            self._theta_dot_prev  = theta_dot_fb
            self._theta_ddot_hat  = 0.0
            self._tau_ext_hat     = 0.0
            self._obs2_initialized = True
            self.get_logger().info('Observer 2 (Force Balance) inizializzato.')

        # Dirty derivative filtrato per stimare θ̈
        alpha_acc = float(self.get_parameter('obs2_alpha_acc').value)
        raw_ddot  = (theta_dot_fb - self._theta_dot_prev) / dt
        self._theta_ddot_hat = (alpha_acc * raw_ddot
                                + (1.0 - alpha_acc) * self._theta_ddot_hat)
        self._theta_dot_prev = theta_dot_fb

        tau_fric_lin = (fric_visc + damping_theta) * theta_dot_fb

        tau_ext_inst = (
            M_eff * self._theta_ddot_hat
            + proj
            + tau_fric_lin
            - tau_m
            - tau_pass
        )
        tau_ext_inst = max(-TAU_MAX, min(TAU_MAX, tau_ext_inst))

        alpha_tau = float(self.get_parameter('obs2_alpha_tau').value)
        self._tau_ext_hat = (alpha_tau * tau_ext_inst
                             + (1.0 - alpha_tau) * self._tau_ext_hat)

        torque_residual = self._tau_ext_meas - self._tau_ext_hat

        # ─────────────────────────────────────────────────────────────────────
        # Filtraggio EMA dei residui
        # ─────────────────────────────────────────────────────────────────────
        RES_MAX = 100.0
        state_residual  = max(-RES_MAX, min(RES_MAX, state_residual))
        torque_residual = max(-RES_MAX, min(RES_MAX, torque_residual))

        self._state_res_filtered  = ema(
            self._state_res_filtered,  state_residual,  alpha)
        self._torque_res_filtered = ema(
            self._torque_res_filtered, torque_residual, alpha)

        self._state_res_buf.append(self._safe_sq(state_residual))
        self._torque_res_buf.append(self._safe_sq(torque_residual))

        # ─────────────────────────────────────────────────────────────────────
        # Pubblicazione
        # ─────────────────────────────────────────────────────────────────────
        msg_sr = Float64()
        msg_sr.data = float(state_residual)
        self._pub_state_res.publish(msg_sr)

        msg_tr = Float64()
        msg_tr.data = float(torque_residual)
        self._pub_torque_res.publish(msg_tr)

        dbg = Float64MultiArray()
        dbg.data = [
            float(self._theta_hat),           # [0]  theta_hat
            float(self._theta_dot_hat),       # [1]  theta_dot_hat
            float(state_residual),            # [2]  state_residual
            float(self._tau_ext_hat),         # [3]  tau_ext_hat
            float(torque_residual),           # [4]  torque_residual
            float(self._state_res_filtered),  # [5]  state_res_filtered
            float(self._torque_res_filtered), # [6]  torque_res_filtered
            float(self._theta_meas),          # [7]  theta_meas
            float(self._theta_dot_meas),      # [8]  theta_dot_meas
            float(self._tau_m),               # [9]  tau_m
            float(self._tau_ext_meas),        # [10] tau_ext_meas
            float(M_eff),                     # [11] M_eff
            float(proj),                      # [12] proj
            float(self._theta_ddot_hat),      # [13] theta_ddot_hat
            float(tau_pass),                  # [14] tau_pass
        ]
        self._pub_debug.publish(dbg)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
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