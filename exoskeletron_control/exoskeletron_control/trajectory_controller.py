#!/usr/bin/env python3
"""
trajectory_controller.py
=========================
Inner-loop trajectory controller per la dinamica ridotta dell'esoscheletro.

Schema:
  - Legge theta / theta_dot da /joint_states  (joint: rev_crank)
  - Riceve riferimenti theta_ref, theta_dot_ref, theta_ddot_ref da /trajectory_ref
  - Calcola azione di controllo PD + feed-forward completo:

      tau_ff = M_eff * theta_ddot_ref
              + proj                          (Coriolis + gravità proiettati)
              - tau_pass_theta                (coppie passive, segno corretto)
              + fric_visc * theta_dot_ref     (compensazione attrito viscoso)
              + damping_theta * theta_dot_ref (compensazione smorzamento)

      tau_pd = Kp * e_theta + Kd * e_theta_dot

      tau_m = tau_pd + tau_ff

  Derivazione dal modello del plant:
  -----------------------------------
  L'equazione ridotta del plant è:

      M_eff * θ̈ = τ_m + τ_pass + τ_ext - proj - τ_fric - τ_damp

  Riordinata per τ_m (ciò che il controller deve generare):

      τ_m = M_eff * θ̈ + proj + τ_fric + τ_damp - τ_pass - τ_ext

  Il FF compensa tutti i termini noti del modello (escluso τ_ext, che è
  l'azione intenzionale dell'utente e non va cancellata):

      τ_ff = M_eff * θ̈_ref + proj - τ_pass + τ_visc_ref + τ_damp_ref

  dove τ_visc_ref e τ_damp_ref sono calcolati sulla velocità di
  RIFERIMENTO (θ̇_ref) per evitare loop algebrici con la misura.

  NOTA: l'attrito Coulomb NON viene compensato nel FF perché è un
  termine non lineare e dissipativo che agisce sulla velocità REALE
  (non su θ̇_ref). Compensarlo su θ̇_ref crea sovracompensazione:
  il FF pompa ±fric_coul Nm in modo quasi binario, il plant accelera
  e l'attrito reale si auto-bilancia, ma il FF continua a spingere.
  Il PD è più adatto a gestire l'attrito Coulomb residuo.

  Nota sui termini di ff_terms dal plant:
    ff_terms[0] = denom = M_eff = B^T M B + Jm
    ff_terms[1] = proj  = B^T (M Ḃ θ̇ + h)     ← usato ora (era g_proj!)
    ff_terms[2] = g_proj = B^T h                ← non più usato
    ff_terms[3] = tau_pass_theta

  Modalità STOP (bridge in 'stop'):
  ----------------------------------
  Quando il bridge entra in modalità 'stop', il controller riceve il topic
  /exo_bridge/mode e passa in modalità holding statica:
    - azzera il termine PD (e_theta, e_theta_dot → 0)
    - azzera il termine inerziale (M_eff * theta_ddot_ref → 0)
    - mantiene SOLO g_proj dell'ultimo ciclo pre-stop come coppia statica
      per contrastare la gravità

  Alla transizione nominal/compliant/torque_limit → stop:
    - congela g_proj al valore corrente (snapshot pre-stop)

  Alla transizione stop → qualsiasi altra modalità:
    - ripristina il comportamento normale (PD+ completo)

Topics
------
  Sub:  /joint_states             sensor_msgs/JointState
        /trajectory_ref           std_msgs/Float64MultiArray
        /exo_dynamics/ff_terms    std_msgs/Float64MultiArray
        /exo_bridge/mode          std_msgs/String
  Pub:  /torque                   std_msgs/Float64
        /traj_ctrl/debug          std_msgs/Float64MultiArray

Parametri ROS2
--------------
  joint_name         (str,   default 'rev_crank')
  Kp                 (float, default 50.0)
  Kd                 (float, default 5.0)
  use_feedforward    (bool,  default True)
  tau_max            (float, default 10.0)
  publish_rate       (float, default 200.0)
  fric_visc          (float, default 2.0)       compensazione attrito viscoso
  damping_theta      (float, default 1.0)       compensazione smorzamento
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray, String
from sensor_msgs.msg import JointState


class TrajectoryController(Node):

    def __init__(self):
        super().__init__('trajectory_controller')

        self.declare_parameter('joint_name',       'rev_crank')
        self.declare_parameter('Kp',                50.0)
        self.declare_parameter('Kd',                 5.0)
        self.declare_parameter('use_feedforward',   True)
        self.declare_parameter('tau_max',           10.0)
        self.declare_parameter('publish_rate',     200.0)

        # ── Parametri compensazione attrito (solo termini lineari) ────
        # Devono corrispondere ai parametri del plant (dynamics_params.yaml).
        # L'attrito Coulomb NON viene compensato nel FF perché è non lineare
        # e dissipativo: compensarlo su theta_dot_ref causa sovracompensazione.
        self.declare_parameter('fric_visc',          2.0)
        self.declare_parameter('damping_theta',      1.0)

        self.joint_name = str(self.get_parameter('joint_name').value)
        publish_rate    = float(self.get_parameter('publish_rate').value)

        # Stato sensori
        self.theta     = 0.0
        self.theta_dot = 0.0
        self.js_received = False

        # Riferimento traiettoria
        self.theta_ref      = 0.0
        self.theta_dot_ref  = 0.0
        self.theta_ddot_ref = 0.0
        self.ref_received   = False

        # Termini feed-forward dal plant
        self.M_eff          = 1.0
        self.proj           = 0.0    # B^T (M Bdot thetadot + h) — era g_proj
        self.g_proj         = 0.0    # B^T h — mantenuto per diagnostica/stop
        self.tau_pass_theta = 0.0
        self.ff_received    = False

        # Uscita corrente
        self.tau_out = 0.0

        # ── Stato modalità bridge ────────────────────────────────────
        self._bridge_mode: str   = 'nominal'
        self._g_proj_at_stop: float = 0.0

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/trajectory_ref', self._ref_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/exo_dynamics/ff_terms', self._ff_terms_cb, 10)
        self.create_subscription(
            String, '/exo_bridge/mode', self._bridge_mode_cb, 10)

        # ── Publishers ───────────────────────────────────────────────
        self.pub_tau  = self.create_publisher(Float64,           '/torque',          10)
        self.pub_diag = self.create_publisher(Float64MultiArray, '/traj_ctrl/debug', 10)

        self.timer = self.create_timer(1.0 / publish_rate, self._control_loop)

        self.get_logger().info(
            f'TrajectoryController avviato | '
            f'Kp={self.get_parameter("Kp").value} '
            f'Kd={self.get_parameter("Kd").value} '
            f'use_feedforward={self.get_parameter("use_feedforward").value} '
            f'fric_visc={self.get_parameter("fric_visc").value} '
            f'damping={self.get_parameter("damping_theta").value}'
        )

    # ── Callbacks ────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return
        if idx < len(msg.position):
            self.theta = float(msg.position[idx])
        if idx < len(msg.velocity):
            self.theta_dot = float(msg.velocity[idx])
        self.js_received = True

    def _ref_cb(self, msg: Float64MultiArray):
        d = msg.data
        self.theta_ref      = float(d[0]) if len(d) > 0 else 0.0
        self.theta_dot_ref  = float(d[1]) if len(d) > 1 else 0.0
        self.theta_ddot_ref = float(d[2]) if len(d) > 2 else 0.0
        self.ref_received   = True

    def _ff_terms_cb(self, msg: Float64MultiArray):
        """
        Layout /exo_dynamics/ff_terms:
          [0] denom  = M_eff = B^T M B + Jm
          [1] proj   = B^T (M Ḃ θ̇ + h)     ← include Coriolis + gravità + Bdot
          [2] g_proj = B^T h                ← solo nonLinearEffects (gravità + Coriolis)
          [3] tau_pass_theta
        """
        d = msg.data
        if len(d) > 0:
            M = float(d[0])
            self.M_eff = M if M > 1e-6 else 1.0
        if len(d) > 1:
            self.proj = float(d[1])
        if len(d) > 2:
            self.g_proj = float(d[2])
        if len(d) > 3:
            self.tau_pass_theta = float(d[3])
        self.ff_received = True

    def _bridge_mode_cb(self, msg: String):
        """
        Riceve la modalità corrente del bridge da /exo_bridge/mode.

        Alla transizione nominal/compliant/torque_limit → stop:
          - congela g_proj al valore corrente (snapshot pre-stop)
          - i successivi aggiornamenti di ff_terms non modificano _g_proj_at_stop

        Alla transizione stop → qualsiasi altra modalità:
          - ripristina il comportamento normale (PD+ completo)
        """
        new_mode = msg.data.strip().lower()
        prev_mode = self._bridge_mode

        if new_mode == prev_mode:
            return

        if new_mode == 'stop' and prev_mode != 'stop':
            self._g_proj_at_stop = self.g_proj
            self.get_logger().warn(
                f'TrajectoryController: bridge → STOP | '
                f'g_proj snapshot={self._g_proj_at_stop:.4f} Nm'
            )

        elif prev_mode == 'stop' and new_mode != 'stop':
            self.get_logger().info(
                f'TrajectoryController: bridge → {new_mode.upper()} | '
                f'ripristino modalità PD+ completa'
            )

        self._bridge_mode = new_mode

    # ── Control loop ─────────────────────────────────────────────────

    def _control_loop(self):
        if not self.js_received or not self.ref_received:
            return

        Kp      = float(self.get_parameter('Kp').value)
        Kd      = float(self.get_parameter('Kd').value)
        use_ff  = bool(self.get_parameter('use_feedforward').value)
        tau_max = float(self.get_parameter('tau_max').value)

        # ── Modalità STOP: holding statica ───────────────────────────
        if self._bridge_mode == 'stop':
            tau_raw = self._g_proj_at_stop
            tau_clamped = float(max(-tau_max, min(tau_max, tau_raw)))
            self.tau_out = tau_clamped

            msg_tau = Float64()
            msg_tau.data = tau_clamped
            self.pub_tau.publish(msg_tau)

            diag = Float64MultiArray()
            diag.data = [
                self.theta_ref,
                self.theta,
                0.0,
                self.theta_dot_ref,
                self.theta_dot,
                0.0,
                0.0,
                self._g_proj_at_stop,
                tau_raw,
                tau_clamped,
                self.M_eff,
                self._g_proj_at_stop,
                self.tau_pass_theta,
            ]
            self.pub_diag.publish(diag)
            return

        # ── Modalità normale: PD + feedforward completo ──────────────

        e_theta     = self.theta_ref     - self.theta
        e_theta_dot = self.theta_dot_ref - self.theta_dot

        tau_pd = Kp * e_theta + Kd * e_theta_dot

        tau_ff = 0.0
        if use_ff and self.ff_received:
            # ── Parametri attrito (solo termini lineari) ───────────
            b_visc  = float(self.get_parameter('fric_visc').value)
            d_theta = float(self.get_parameter('damping_theta').value)

            # Compensazione attrito lineare sulla velocità di RIFERIMENTO
            # (evita loop algebrico con la misura).
            # L'attrito Coulomb NON viene compensato: è non lineare,
            # agisce su θ̇ reale (non θ̇_ref), e compensarlo in FF
            # causa sovracompensazione con gradini di ±fric_coul Nm.
            tau_visc_ff = b_visc * self.theta_dot_ref
            tau_damp_ff = d_theta * self.theta_dot_ref

            # ── Feedforward ──────────────────────────────────────────
            #
            # Dal modello del plant:
            #   τ_m = M_eff·θ̈ + proj + τ_fric + τ_damp - τ_pass - τ_ext
            #
            # FF (senza τ_ext né Coulomb):
            #   τ_ff = M_eff·θ̈_ref + proj - τ_pass + τ_visc_ref + τ_damp_ref
            #
            tau_ff = (
                self.M_eff * self.theta_ddot_ref
                + self.proj                  # B^T(M·Ḃ·θ̇ + h): gravità + Coriolis + Bdot
                - self.tau_pass_theta        # segno CORRETTO: il plant lo somma, noi lo sottraiamo
                + tau_visc_ff                # compensazione attrito viscoso (lineare)
                + tau_damp_ff                # compensazione smorzamento (lineare)
            )

        tau_raw = tau_pd + tau_ff
        tau_clamped = float(max(-tau_max, min(tau_max, tau_raw)))
        self.tau_out = tau_clamped

        msg_tau = Float64()
        msg_tau.data = tau_clamped
        self.pub_tau.publish(msg_tau)

        diag = Float64MultiArray()
        diag.data = [
            self.theta_ref,
            self.theta,
            e_theta,
            self.theta_dot_ref,
            self.theta_dot,
            e_theta_dot,
            tau_pd,
            tau_ff,
            tau_raw,
            tau_clamped,
            self.M_eff,
            self.g_proj,
            self.tau_pass_theta,
        ]
        self.pub_diag.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()