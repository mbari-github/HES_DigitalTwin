#!/usr/bin/env python3
"""
trajectory_controller.py
=========================
Inner-loop trajectory controller per la dinamica ridotta dell'esoscheletro.

Schema:
  - Legge theta / theta_dot da /joint_states  (joint: rev_crank)
  - Riceve riferimenti theta_ref, theta_dot_ref, theta_ddot_ref da /trajectory_ref
  - Calcola azione di controllo PD+ con feed-forward di gravità:
      tau = M_eff * (theta_ddot_ref + Kp*(theta_ref - theta) + Kd*(theta_dot_ref - theta_dot))
            + g_proj + tau_pass_theta
  - Pubblica tau su /torque  (Float64)

  Modalità STOP (bridge in 'stop'):
  ----------------------------------
  Quando il bridge entra in modalità 'stop', il controller riceve il topic
  /exo_bridge/mode e passa in modalità holding statica:
    - azzera il termine PD (e_theta, e_theta_dot → 0)
    - azzera il termine inerziale (M_eff * theta_ddot_ref → 0)
    - mantiene SOLO g_proj dell'ultimo ciclo pre-stop come coppia statica
      per contrastare la gravità

  Questo previene il problema in cui theta è congelato da fault injection
  (canale 3) ma i ff_terms (g_proj, tau_pass_theta) continuano ad aggiornarsi
  con la q reale del plant: in quel caso tau_ff sarebbe non nulla anche con
  e_theta = 0, producendo una coppia inattesa passata invariata dal bridge
  in hold_only.

Topics:
  Sub:  /joint_states             sensor_msgs/JointState
        /trajectory_ref           std_msgs/Float64MultiArray
        /exo_dynamics/ff_terms    std_msgs/Float64MultiArray
        /exo_bridge/mode          std_msgs/String
  Pub:  /torque                   std_msgs/Float64
        /traj_ctrl/debug          std_msgs/Float64MultiArray

Parametri ROS2:
  joint_name        (str,   default 'rev_crank')
  Kp                (float, default 50.0)
  Kd                (float, default 5.0)
  use_feedforward   (bool,  default True)
  tau_max           (float, default 10.0)
  publish_rate      (float, default 200.0)
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray, String
from sensor_msgs.msg import JointState


class TrajectoryController(Node):

    def __init__(self):
        super().__init__('trajectory_controller')

        self.declare_parameter('joint_name',    'rev_crank')
        self.declare_parameter('Kp',             50.0)
        self.declare_parameter('Kd',              5.0)
        self.declare_parameter('use_feedforward', True)
        self.declare_parameter('tau_max',         10.0)
        self.declare_parameter('publish_rate',   200.0)

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
        self.g_proj         = 0.0
        self.tau_pass_theta = 0.0
        self.ff_received    = False

        # Uscita corrente
        self.tau_out = 0.0

        # ── Stato modalità bridge ────────────────────────────────────
        # Quando il bridge è in 'stop', il controller passa in holding
        # statica: azzera PD e inerziale, usa solo g_proj congelato.
        self._bridge_mode: str   = 'nominal'
        # g_proj snapshot al momento dell'ingresso in stop:
        # congela il termine gravitazionale alla configurazione pre-stop,
        # indipendentemente da eventuali aggiornamenti successivi di ff_terms.
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
            f'use_feedforward={self.get_parameter("use_feedforward").value}'
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
        d = msg.data
        if len(d) > 0:
            M = float(d[0])
            self.M_eff = M if M > 1e-6 else 1.0
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
            # Snapshot di g_proj al momento dell'ingresso in stop
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
        # Quando il bridge è in stop, non usare il termine PD né
        # il termine inerziale. Usare solo il g_proj snapshot pre-stop
        # per contrastare la gravità staticamente.
        #
        # Motivazione: se theta è congelato da fault injection (canale 3),
        # e_theta = 0 ma g_proj e tau_pass_theta continuano ad aggiornarsi
        # con la q reale del plant. Passare tau_ff variabile al bridge in
        # hold_only produrrebbe una coppia inattesa che muove il dito.
        if self._bridge_mode == 'stop':
            tau_raw = self._g_proj_at_stop
            tau_clamped = float(max(-tau_max, min(tau_max, tau_raw)))
            self.tau_out = tau_clamped

            msg_tau = Float64()
            msg_tau.data = tau_clamped
            self.pub_tau.publish(msg_tau)

            # Diagnostica in stop: campi PD e ff variabili a zero,
            # tau_ff mostra il g_proj snapshot usato
            diag = Float64MultiArray()
            diag.data = [
                self.theta_ref,
                self.theta,
                0.0,                    # e_theta forzato a 0 in stop
                self.theta_dot_ref,
                self.theta_dot,
                0.0,                    # e_theta_dot forzato a 0 in stop
                0.0,                    # tau_pd = 0
                self._g_proj_at_stop,   # tau_ff = solo g_proj snapshot
                tau_raw,
                tau_clamped,
                self.M_eff,
                self._g_proj_at_stop,
                self.tau_pass_theta,
            ]
            self.pub_diag.publish(diag)
            return

        # ── Modalità normale: PD+ completo ───────────────────────────
        e_theta     = self.theta_ref     - self.theta
        e_theta_dot = self.theta_dot_ref - self.theta_dot

        tau_pd = Kp * e_theta + Kd * e_theta_dot

        tau_ff = 0.0
        if use_ff and self.ff_received:
            tau_ff = (
                self.M_eff * self.theta_ddot_ref
                + self.g_proj
                + self.tau_pass_theta
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