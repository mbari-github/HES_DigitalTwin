#!/usr/bin/env python3
"""
Inner-loop trajectory controller for the reduced dynamics of the exoskeleton.

Architecture:
  - Reads theta / theta_dot from /joint_states  (joint: rev_crank)
  - Receives references theta_ref, theta_dot_ref, theta_ddot_ref from /trajectory_ref
  - Computes control action as PD + full feed-forward:

      tau_ff = M_eff * theta_ddot_ref
              + proj                          (Coriolis + gravity projected)
              - tau_pass_theta                (passive torques, correct sign)
              + fric_visc * theta_dot_ref     (viscous friction compensation)
              + damping_theta * theta_dot_ref (damping compensation)

      tau_pd = Kp * e_theta + Kd * e_theta_dot

      tau_m = tau_pd + tau_ff

Feed-forward derivation from the plant model:
----------------------------------------------
The reduced plant equation is:

    M_eff * θ̈ = τ_m + τ_pass + τ_ext - proj - τ_fric - τ_damp

Rearranged for τ_m (what the controller must generate):

    τ_m = M_eff * θ̈ + proj + τ_fric + τ_damp - τ_pass - τ_ext

The FF compensates all known model terms (excluding τ_ext, which is the
intentional user action and must not be cancelled):

    τ_ff = M_eff * θ̈_ref + proj - τ_pass + τ_visc_ref + τ_damp_ref

where τ_visc_ref and τ_damp_ref are computed on the REFERENCE velocity
(θ̇_ref) to avoid algebraic loops with the measurement.

NOTE: Coulomb friction is NOT compensated in the FF because it is a
nonlinear, dissipative term that acts on the REAL velocity (not θ̇_ref).
Compensating it on θ̇_ref causes overcompensation: the FF injects ±fric_coul Nm
in a nearly binary fashion, the plant accelerates and the real friction
self-balances, but the FF keeps pushing. The PD is better suited to handle
the residual Coulomb friction.

ff_terms layout from the plant (/exo_dynamics/ff_terms):
    ff_terms[0] = denom = M_eff = B^T M B + Jm
    ff_terms[1] = proj  = B^T (M Ḃ θ̇ + h)     ← used (previously labelled g_proj)
    ff_terms[2] = g_proj = B^T h                ← not used in FF (kept for diagnostics/stop)
    ff_terms[3] = tau_pass_theta

STOP mode (bridge in 'stop'):
------------------------------
When the bridge enters 'stop' mode, the controller receives the topic
/exo_bridge/mode and switches to static holding:
  - zeros the PD term (e_theta, e_theta_dot → 0)
  - zeros the inertial term (M_eff * theta_ddot_ref → 0)
  - applies ONLY g_proj from the last pre-stop cycle as a static torque
    to counteract gravity

On transition nominal/compliant/torque_limit → stop:
  - g_proj is frozen at the current value (pre-stop snapshot)

On transition stop → any other mode:
  - full PD+ behaviour is restored

Topics
------
  Sub:  /joint_states             sensor_msgs/JointState
        /trajectory_ref           exoskeleton_safety_msgs/Float64ArrayStamped
        /exo_dynamics/ff_terms    exoskeleton_safety_msgs/Float64ArrayStamped
        /exo_bridge/mode          std_msgs/String
  Pub:  /torque                   exoskeleton_safety_msgs/Float64Stamped
        /traj_ctrl/debug          exoskeleton_safety_msgs/Float64ArrayStamped

ROS2 Parameters
---------------
  joint_name         (str,   default 'rev_crank')
  Kp                 (float, default 50.0)
  Kd                 (float, default 5.0)
  use_feedforward    (bool,  default True)
  tau_max            (float, default 10.0)
  publish_rate       (float, default 200.0)
  fric_visc          (float, default 2.0)       viscous friction compensation
  damping_theta      (float, default 1.0)       damping compensation
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from exoskeleton_safety_msgs.msg import Float64Stamped, Float64ArrayStamped
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

        # ── Friction compensation parameters (linear terms only) ──────
        # Must match the plant parameters in dynamics_params.yaml.
        # Coulomb friction is NOT compensated in the FF because it is nonlinear
        # and dissipative: compensating it on theta_dot_ref causes overcompensation.
        self.declare_parameter('fric_visc',          2.0)
        self.declare_parameter('damping_theta',      1.0)

        self.joint_name = str(self.get_parameter('joint_name').value)
        publish_rate    = float(self.get_parameter('publish_rate').value)

        # Sensor state
        self.theta     = 0.0
        self.theta_dot = 0.0
        self.js_received = False

        # Trajectory reference
        self.theta_ref      = 0.0
        self.theta_dot_ref  = 0.0
        self.theta_ddot_ref = 0.0
        self.ref_received   = False

        # Feed-forward terms from the plant
        self.M_eff          = 1.0
        self.proj           = 0.0    # B^T (M Bdot thetadot + h) — Coriolis + gravity + Bdot
        self.g_proj         = 0.0    # B^T h — kept for diagnostics and STOP mode
        self.tau_pass_theta = 0.0
        self.ff_received    = False

        # Current output torque
        self.tau_out = 0.0

        # ── Bridge mode state ────────────────────────────────────────
        self._bridge_mode: str   = 'nominal'
        self._g_proj_at_stop: float = 0.0  # gravity snapshot taken at STOP entry

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(
            Float64ArrayStamped, '/trajectory_ref', self._ref_cb, 10)
        self.create_subscription(
            Float64ArrayStamped, '/exo_dynamics/ff_terms', self._ff_terms_cb, 10)
        self.create_subscription(
            String, '/exo_bridge/mode', self._bridge_mode_cb, 10)

        # ── Publishers ───────────────────────────────────────────────
        self.pub_tau  = self.create_publisher(Float64Stamped,      '/torque',          10)
        self.pub_diag = self.create_publisher(Float64ArrayStamped, '/traj_ctrl/debug', 10)

        self.timer = self.create_timer(1.0 / publish_rate, self._control_loop)

        self.get_logger().info(
            f'TrajectoryController started | '
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

    def _ref_cb(self, msg: Float64ArrayStamped):
        d = msg.data
        self.theta_ref      = float(d[0]) if len(d) > 0 else 0.0
        self.theta_dot_ref  = float(d[1]) if len(d) > 1 else 0.0
        self.theta_ddot_ref = float(d[2]) if len(d) > 2 else 0.0
        self.ref_received   = True

    def _ff_terms_cb(self, msg: Float64ArrayStamped):
        """
        Layout of /exo_dynamics/ff_terms:
          [0] denom  = M_eff = B^T M B + Jm
          [1] proj   = B^T (M Ḃ θ̇ + h)     ← includes Coriolis + gravity + Bdot term
          [2] g_proj = B^T h                ← nonLinearEffects only (gravity + Coriolis)
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
        Receives the current bridge mode from /exo_bridge/mode.

        On transition nominal/compliant/torque_limit → stop:
          - g_proj is frozen at the current value (pre-stop snapshot)
          - subsequent ff_terms updates do not change _g_proj_at_stop

        On transition stop → any other mode:
          - full PD+ behaviour is restored
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
                f'resuming full PD+ mode'
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

        # ── STOP mode: static gravity holding ────────────────────────
        if self._bridge_mode == 'stop':
            tau_raw = self._g_proj_at_stop
            tau_clamped = float(max(-tau_max, min(tau_max, tau_raw)))
            self.tau_out = tau_clamped

            stamp = self.get_clock().now().to_msg()
            msg_tau = Float64Stamped()
            msg_tau.header.stamp = stamp
            msg_tau.data = tau_clamped
            self.pub_tau.publish(msg_tau)

            diag = Float64ArrayStamped()
            diag.header.stamp = stamp
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

        # ── Normal mode: PD + full feed-forward ──────────────────────

        e_theta     = self.theta_ref     - self.theta
        e_theta_dot = self.theta_dot_ref - self.theta_dot

        tau_pd = Kp * e_theta + Kd * e_theta_dot

        tau_ff = 0.0
        if use_ff and self.ff_received:
            # ── Linear friction compensation parameters ────────────
            b_visc  = float(self.get_parameter('fric_visc').value)
            d_theta = float(self.get_parameter('damping_theta').value)

            # Compensate linear friction on REFERENCE velocity to avoid
            # algebraic loops with the measurement.
            # Coulomb friction is NOT compensated: it is nonlinear, acts on
            # real θ̇ (not θ̇_ref), and compensating it in FF causes
            # overcompensation with ±fric_coul Nm step disturbances.
            tau_visc_ff = b_visc * self.theta_dot_ref
            tau_damp_ff = d_theta * self.theta_dot_ref

            # ── Feed-forward torque ───────────────────────────────────
            #
            # From the plant model:
            #   τ_m = M_eff·θ̈ + proj + τ_fric + τ_damp - τ_pass - τ_ext
            #
            # FF (without τ_ext and without Coulomb friction):
            #   τ_ff = M_eff·θ̈_ref + proj - τ_pass + τ_visc_ref + τ_damp_ref
            #
            tau_ff = (
                self.M_eff * self.theta_ddot_ref
                + self.proj                  # B^T(M·Ḃ·θ̇ + h): gravity + Coriolis + Bdot
                - self.tau_pass_theta        # correct sign: plant adds it, we subtract it
                + tau_visc_ff                # viscous friction compensation (linear)
                + tau_damp_ff                # damping compensation (linear)
            )

        tau_raw = tau_pd + tau_ff
        tau_clamped = float(max(-tau_max, min(tau_max, tau_raw)))
        self.tau_out = tau_clamped

        stamp = self.get_clock().now().to_msg()
        msg_tau = Float64Stamped()
        msg_tau.header.stamp = stamp
        msg_tau.data = tau_clamped
        self.pub_tau.publish(msg_tau)

        diag = Float64ArrayStamped()
        diag.header.stamp = stamp
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
