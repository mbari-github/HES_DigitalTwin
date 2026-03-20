#!/usr/bin/env python3
"""

Inner-loop trajectory controller per la dinamica ridotta dell'esoscheletro.

Schema:
  - Legge theta / theta_dot da /joint_states  (joint: rev_crank)
  - Riceve riferimenti theta_ref, theta_dot_ref, theta_ddot_ref da /trajectory_ref
      msg type: Float64MultiArray  [theta_ref, theta_dot_ref, theta_ddot_ref]
  - Calcola azione di controllo PD+ con feed-forward di gravità:
      tau = M_eff * (theta_ddot_ref + Kp*(theta_ref - theta) + Kd*(theta_dot_ref - theta_dot))
            + g_proj + tau_pass_theta
  - Pubblica tau su /torque  (Float64)

Dove M_eff, g_proj, tau_pass_theta (e opzionalmente altri termini) vengono letti dal topic /exo_dynamics/ff_terms
    msg type: Float64MultiArray con ordine stabile:
      ff_terms[0] = M_eff          (massa inerziale ridotta su theta)
      ff_terms[1] = proj           (B^T*(M*Bdot*theta_dot + h) - opzionale qui)
      ff_terms[2] = g_proj         (proiezione gravità su theta)
      ff_terms[3] = tau_pass_theta (torque passivo proiettato su theta)

In alternativa, se non si vuole dipendere dal topic feed-forward, i guadagni compensano, se non si vuole dipendere dal debug topic, i guadagni compensano
direttamente (modalità "PD puro" con Kff=0).

Topics:
  Sub:  /joint_states          sensor_msgs/JointState
        /trajectory_ref        std_msgs/Float64MultiArray  [theta_ref, theta_dot_ref, theta_ddot_ref]
        /exo_dynamics/ff_terms    std_msgs/Float64MultiArray  (opzionale, per feed-forward)
  Pub:  /torque                std_msgs/Float64
        /traj_ctrl/debug       std_msgs/Float64MultiArray  (diagnostica)

Parametri ROS2:
  joint_name        (str,   default 'rev_crank')
  Kp                (float, default 50.0)   [Nm/rad]
  Kd                (float, default 5.0)    [Nm*s/rad]
  use_feedforward   (bool,  default True)   abilita M_eff * ddot_ref + g_proj + tau_pass
  tau_max           (float, default 10.0)   saturazione coppia [Nm]
  publish_rate      (float, default 200.0)  Hz del loop di controllo
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


class TrajectoryController(Node):

    def __init__(self):
        super().__init__('trajectory_controller')

        # ------------------------------------------------
        # Parametri
        # ------------------------------------------------
        self.declare_parameter('joint_name',      'rev_crank')
        self.declare_parameter('Kp',              50.0)
        self.declare_parameter('Kd',               5.0)
        self.declare_parameter('use_feedforward',  True)
        self.declare_parameter('tau_max',          10.0)
        self.declare_parameter('publish_rate',    200.0)

        self.joint_name     = str(self.get_parameter('joint_name').value)
        self.Kp             = float(self.get_parameter('Kp').value)
        self.Kd             = float(self.get_parameter('Kd').value)
        self.use_ff         = bool(self.get_parameter('use_feedforward').value)
        self.tau_max        = float(self.get_parameter('tau_max').value)
        publish_rate        = float(self.get_parameter('publish_rate').value)

        # ------------------------------------------------
        # Stato
        # ------------------------------------------------
        # Stato attuale (dal plant)
        self.theta          = 0.0
        self.theta_dot      = 0.0
        self.js_received    = False

        # Riferimento
        self.theta_ref      = 0.0
        self.theta_dot_ref  = 0.0
        self.theta_ddot_ref = 0.0
        self.ref_received   = False

        self.M_eff          = 1.0   # fallback: unitario (guadagno PD puro)
        self.g_proj         = 0.0
        self.tau_pass_theta = 0.0
        self.ff_received = False

        # Ultimo tau calcolato (per publish e diagnostica)
        self.tau_out        = 0.0

        # ------------------------------------------------
        # ROS I/O
        # ------------------------------------------------
        self.sub_js = self.create_subscription(
            JointState,
            '/joint_states',
            self._js_cb,
            10
        )
        self.sub_ref = self.create_subscription(
            Float64MultiArray,
            '/trajectory_ref',
            self._ref_cb,
            10
        )
        self.sub_ff = self.create_subscription(
            Float64MultiArray,
            '/exo_dynamics/ff_terms',
            self._ff_terms_cb,
            10
        )

        self.pub_tau = self.create_publisher(Float64, '/torque', 10)
        self.pub_diag = self.create_publisher(
            Float64MultiArray, '/traj_ctrl/debug', 10
        )

        dt_ctrl = 1.0 / publish_rate
        self.timer = self.create_timer(dt_ctrl, self._control_loop)

        self.get_logger().info(
            f"TrajectoryController avviato | joint={self.joint_name} | "
            f"Kp={self.Kp} Kd={self.Kd} use_ff={self.use_ff} tau_max={self.tau_max}"
        )

    # ------------------------------------------------
    # Callbacks
    # ------------------------------------------------
    def _js_cb(self, msg: JointState):
        """Legge theta e theta_dot dal joint 'rev_crank' in /joint_states."""
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
        """
        Riceve il riferimento di traiettoria come Float64MultiArray.
        Layout atteso: [theta_ref, theta_dot_ref, theta_ddot_ref]
        Se vengono forniti meno di 3 elementi, i restanti restano a 0.
        """
        d = msg.data
        self.theta_ref      = float(d[0]) if len(d) > 0 else 0.0
        self.theta_dot_ref  = float(d[1]) if len(d) > 1 else 0.0
        self.theta_ddot_ref = float(d[2]) if len(d) > 2 else 0.0
        self.ref_received   = True

    def _ff_terms_cb(self, msg: Float64MultiArray):
        """
        Legge i termini feed-forward dal plant su /exo_dynamics/ff_terms.

        Layout atteso (ordine stabile):
          [0] M_eff          = B^T M B + Jm
          [1] proj           = B^T*(M*Bdot*theta_dot + h)   (non usato qui)
          [2] g_proj         = proiezione gravità su theta
          [3] tau_pass_theta = coppia passiva proiettata su theta
        """
        d = msg.data
        if len(d) > 0:
            M = float(d[0])
            self.M_eff = M if M > 1e-6 else 1.0
        if len(d) > 2:
            self.g_proj = float(d[2])
        if len(d) > 3:
            self.tau_pass_theta = float(d[3])
        self.ff_received = True
    # ------------------------------------------------
    # Control loop
    # ------------------------------------------------
    def _control_loop(self):
        """
        Legge parametri (aggiornabili a runtime), calcola PD+ e pubblica su /torque.
        """
        # Lettura parametri (possono essere cambiati dinamicamente via ros2 param set)
        Kp      = float(self.get_parameter('Kp').value)
        Kd      = float(self.get_parameter('Kd').value)
        use_ff  = bool(self.get_parameter('use_feedforward').value)
        tau_max = float(self.get_parameter('tau_max').value)

        # Errori
        e_theta     = self.theta_ref     - self.theta
        e_theta_dot = self.theta_dot_ref - self.theta_dot

        # Termine PD
        tau_pd = Kp * e_theta + Kd * e_theta_dot

        # Termine feed-forward (solo se abilitato e dati disponibili)
        tau_ff = 0.0
        if use_ff and self.ff_received:
            # M_eff * theta_ddot_ref: termine inerziale
            # g_proj: compensazione gravità (il plant lo sottrae internamente, lo rimettiamo)
            # tau_pass_theta: compensazione passivo (idem)
            tau_ff = (
                self.M_eff * self.theta_ddot_ref
                + self.g_proj
                + self.tau_pass_theta
            )

        tau_raw = tau_pd + tau_ff

        # Saturazione
        tau_clamped = float(max(-tau_max, min(tau_max, tau_raw)))
        self.tau_out = tau_clamped

        # Pubblica coppia
        msg_tau = Float64()
        msg_tau.data = tau_clamped
        self.pub_tau.publish(msg_tau)

        # Diagnostica
        # [0] theta_ref         [1] theta       [2] e_theta
        # [3] theta_dot_ref     [4] theta_dot   [5] e_theta_dot
        # [6] tau_pd            [7] tau_ff      [8] tau_raw    [9] tau_out
        # [10] M_eff            [11] g_proj     [12] tau_pass_theta
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