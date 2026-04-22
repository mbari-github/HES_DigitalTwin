#!/usr/bin/env python3
"""
Outer-loop admittance controller for the reduced dynamics of the exoskeleton.

Working principle
-----------------
The admittance controller implements a virtual mass-damper-spring model that
converts the user force into a trajectory reference for the inner loop.

The input force is tau_ext_theta [Nm], published by the dynamics node on
/exo_dynamics/tau_ext_theta. This scalar already represents the correct
projection of the user wrench onto the active DOF (theta = rev_crank):

    tau_ext_theta = B^T * J^T * W_user

where B = dq/dtheta is the instantaneous kinematic vector and J is the
Jacobian of the contact frame. The projection is therefore updated at each
step by the plant, taking into account the current mechanism configuration.

Virtual model
-------------
    M * θ̈_v + D * θ̇_v + K * (θ_v - θ_eq) = tau_ext_theta

    θ̈_v = (tau_ext_theta - D * θ̇_v - K * (θ_v - θ_eq)) / M

Explicit Euler integration:
    θ̇_v  ← θ̇_v + θ̈_v * dt
    θ_v  ← θ_v + θ̇_v * dt

The output [θ_v, θ̇_v, θ̈_v] is published on /trajectory_ref and consumed
directly by the inner loop (trajectory_controller.py).

Block diagram
-------------
  /exo_dynamics/tau_ext_theta  →  [M-D-K model]  →  /trajectory_ref
  /joint_states                →  (init θ_v and resume from freeze)
  /admittance/freeze           →  (freeze / unfreeze integration)

Topics
------
  Sub:  /exo_dynamics/tau_ext_theta   exoskeletron_safety_msgs/Float64Stamped
        /joint_states                 sensor_msgs/JointState
        /admittance/freeze            std_msgs/Bool
  Pub:  /trajectory_ref               exoskeletron_safety_msgs/Float64ArrayStamped  [θ_ref, θ̇_ref, θ̈_ref]
        /admittance/debug             exoskeletron_safety_msgs/Float64ArrayStamped

Virtual wall (position barrier)
--------------------------------
When theta_v enters the buffer zone of width wall_buffer before a mechanical
limit, a virtual spring-damper pushes the trajectory back inward. This ensures
that velocity and acceleration remain physically consistent with position,
preventing the trajectory controller from driving the motor against the hard stop.
The rigid clamp is maintained as a last-resort safety net; if triggered,
velocity and acceleration are zeroed.

    tau_wall = -K_wall * penetration - D_wall * vel_toward_wall

The force tau_wall enters the virtual model balance together with tau_in,
tau_spring and tau_damper.

ROS2 Parameters
---------------
  joint_name     (str,   default 'rev_crank')
  M_virt         (float, default 0.5)
  D_virt         (float, default 5.0)
  K_virt         (float, default 2.0)
  theta_eq       (float, default 0.0)
  force_deadband (float, default 0.1)
  theta_ref_min  (float, default -0.75)   lower trajectory limit [rad]
  theta_ref_max  (float, default  0.09)   upper trajectory limit [rad]
  theta_dot_max  (float, default  5.0)
  wall_buffer    (float, default  0.05)   barrier zone width [rad]
  K_wall         (float, default 80.0)    barrier stiffness [Nm/rad]
  D_wall         (float, default 10.0)    barrier damping [Nm·s/rad]
  publish_rate   (float, default 200.0)
  init_from_js   (bool,  default True)
"""

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from exoskeletron_safety_msgs.msg import Float64Stamped, Float64ArrayStamped
from sensor_msgs.msg import JointState


class AdmittanceController(Node):

    def __init__(self):
        super().__init__('admittance_controller')

        self.declare_parameter('joint_name',     'rev_crank')
        self.declare_parameter('M_virt',          0.5)
        self.declare_parameter('D_virt',          5.0)
        self.declare_parameter('K_virt',          2.0)
        self.declare_parameter('theta_eq',        0.0)
        self.declare_parameter('force_deadband',  0.0)
        self.declare_parameter('theta_ref_min',  -0.75)
        self.declare_parameter('theta_ref_max',   0.09)
        self.declare_parameter('theta_dot_max',   5.0)
        self.declare_parameter('wall_buffer',      0.05)
        self.declare_parameter('K_wall',          80.0)
        self.declare_parameter('D_wall',          10.0)
        self.declare_parameter('publish_rate',  200.0)
        self.declare_parameter('init_from_js',   True)

        publish_rate    = float(self.get_parameter('publish_rate').value)
        self.joint_name = str(self.get_parameter('joint_name').value)
        self.init_from_js = bool(self.get_parameter('init_from_js').value)

        # Virtual model internal state
        self.theta_v      = 0.0
        self.theta_dot_v  = 0.0
        self.theta_ddot_v = 0.0

        # Flag: set to True once theta_v has been seeded from /joint_states
        self._initialized = not self.init_from_js

        # Freeze state:
        #   True  → integration is suspended; theta_v and theta_dot_v are held constant
        #   False → normal operation
        # The bridge publishes freeze=True when entering STOP mode, and freeze=False
        # when leaving it. On resume, theta_v is realigned to the real joint position
        # to avoid a torque spike caused by drift accumulated during the freeze.
        self._frozen = False

        # Last known real joint position (used to realign theta_v on resume from freeze)
        self._last_real_theta: float | None = None

        self.tau_ext_theta  = 0.0
        self.force_received = False

        self._dt = 1.0 / publish_rate

        # ── Subscribers ──────────────────────────────────────────────
        self.sub_force = self.create_subscription(
            Float64Stamped,
            '/exo_dynamics/tau_ext_theta',
            self._force_cb,
            10
        )
        self.sub_js = self.create_subscription(
            JointState,
            '/joint_states',
            self._js_cb,
            10
        )
        # Freeze topic published by the bridge when entering/leaving STOP mode
        self.sub_freeze = self.create_subscription(
            Bool,
            '/admittance/freeze',
            self._freeze_cb,
            10
        )

        self.pub_ref  = self.create_publisher(Float64ArrayStamped, '/trajectory_ref', 10)
        self.pub_diag = self.create_publisher(Float64ArrayStamped, '/admittance/debug', 10)

        self.timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"AdmittanceController started | "
            f"M={self.get_parameter('M_virt').value} "
            f"D={self.get_parameter('D_virt').value} "
            f"K={self.get_parameter('K_virt').value} "
            f"theta_eq={self.get_parameter('theta_eq').value} "
            f"init_from_js={self.init_from_js}"
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def _js_cb(self, msg: JointState):
        """
        Seeds theta_v from the current crank position on the first call.
        Also tracks _last_real_theta on every tick so it is available on resume.
        """
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return

        if idx < len(msg.position):
            self._last_real_theta = float(msg.position[idx])

            if not self._initialized:
                self.theta_v     = self._last_real_theta
                self.theta_dot_v = 0.0
                self._initialized = True
                self.get_logger().info(
                    f"theta_v initialized from /joint_states: {self.theta_v:.4f} rad"
                )

    def _force_cb(self, msg: Float64Stamped):
        self.tau_ext_theta = float(msg.data)
        self.force_received = True

    def _freeze_cb(self, msg: Bool):
        """
        Receives the freeze/unfreeze command from the bridge.

        freeze=True  → suspend integration (bridge entered STOP).
                        theta_v and theta_dot_v are held at their current value.
        freeze=False → resume integration (bridge exited STOP).
                        On resume, theta_v is reset to the current real position
                        to avoid a torque spike caused by drift accumulated during freeze.
        """
        new_frozen = bool(msg.data)

        if new_frozen == self._frozen:
            return  # no state change

        if new_frozen:
            self._frozen = True
            self.get_logger().warn(
                f'AdmittanceController: FREEZE | theta_v={self.theta_v:.4f} rad'
            )
        else:
            # On resume: realign theta_v to real position to avoid torque spike
            if self._last_real_theta is not None:
                old_theta_v = self.theta_v
                self.theta_v     = self._last_real_theta
                self.theta_dot_v = 0.0
                self.get_logger().info(
                    f'AdmittanceController: UNFREEZE | '
                    f'theta_v realigned: {old_theta_v:.4f} → {self.theta_v:.4f} rad'
                )
            else:
                self.get_logger().warn(
                    'AdmittanceController: UNFREEZE | _last_real_theta not available, '
                    'theta_v unchanged'
                )
            self._frozen = False

    # ============================================================
    # CONTROL LOOP
    # ============================================================

    def _control_loop(self):
        if not self._initialized:
            return

        # When the bridge is in STOP, do not integrate.
        # Still publish the frozen reference to keep the topic alive
        # (prevents the trajectory controller watchdog from triggering).
        if self._frozen:
            ref = Float64ArrayStamped()
            ref.header.stamp = self.get_clock().now().to_msg()
            ref.data = [float(self.theta_v), 0.0, 0.0]
            self.pub_ref.publish(ref)
            return

        M        = float(self.get_parameter('M_virt').value)
        D        = float(self.get_parameter('D_virt').value)
        K        = float(self.get_parameter('K_virt').value)
        theta_eq = float(self.get_parameter('theta_eq').value)
        db       = float(self.get_parameter('force_deadband').value)
        th_min   = float(self.get_parameter('theta_ref_min').value)
        th_max   = float(self.get_parameter('theta_ref_max').value)
        dv_max   = float(self.get_parameter('theta_dot_max').value)
        w_buf    = float(self.get_parameter('wall_buffer').value)
        K_w      = float(self.get_parameter('K_wall').value)
        D_w      = float(self.get_parameter('D_wall').value)

        tau_in = self.tau_ext_theta
        if abs(tau_in) < db:
            tau_in = 0.0

        # ── Virtual wall (position barrier) ───────────────────────────
        # Repulsive spring-damper force when theta_v enters the buffer zone
        # of width w_buf before the mechanical limits.
        # The damper acts only on the velocity component directed toward the
        # wall (unilateral), so it does not brake spontaneous return inward.
        tau_wall = 0.0
        upper_threshold = th_max - w_buf
        lower_threshold = th_min + w_buf

        if self.theta_v > upper_threshold:
            penetration = self.theta_v - upper_threshold
            vel_into_wall = max(self.theta_dot_v, 0.0)
            tau_wall = -K_w * penetration - D_w * vel_into_wall

        elif self.theta_v < lower_threshold:
            penetration = lower_threshold - self.theta_v
            vel_into_wall = max(-self.theta_dot_v, 0.0)
            tau_wall = K_w * penetration + D_w * vel_into_wall

        # ── M-D-K virtual dynamics with barrier ──────────────────────
        tau_spring = K * (self.theta_v - theta_eq)
        tau_damper = D * self.theta_dot_v

        if M < 1e-6:
            if D < 1e-6:
                self.theta_dot_v  = 0.0
                self.theta_ddot_v = 0.0
            else:
                self.theta_dot_v  = (tau_in + tau_wall - tau_spring) / D
                self.theta_ddot_v = 0.0
        else:
            self.theta_ddot_v = (tau_in + tau_wall - tau_damper - tau_spring) / M

        # ── Explicit Euler integration ────────────────────────────────
        self.theta_dot_v += self.theta_ddot_v * self._dt
        self.theta_dot_v  = float(np.clip(self.theta_dot_v, -dv_max, dv_max))
        self.theta_v     += self.theta_dot_v * self._dt

        # ── Hard clamp (last-resort safety net) ──────────────────────
        # If theta_v exceeds the limits despite the virtual wall, clamp the
        # position and zero out velocity/acceleration so the trajectory
        # controller does not push the motor against the hard stop.
        if self.theta_v >= th_max:
            self.theta_v      = float(th_max)
            self.theta_dot_v  = min(self.theta_dot_v, 0.0)
            self.theta_ddot_v = min(self.theta_ddot_v, 0.0)
        elif self.theta_v <= th_min:
            self.theta_v      = float(th_min)
            self.theta_dot_v  = max(self.theta_dot_v, 0.0)
            self.theta_ddot_v = max(self.theta_ddot_v, 0.0)

        stamp = self.get_clock().now().to_msg()
        ref = Float64ArrayStamped()
        ref.header.stamp = stamp
        ref.data = [
            float(self.theta_v),
            float(self.theta_dot_v),
            float(self.theta_ddot_v),
        ]
        self.pub_ref.publish(ref)

        diag = Float64ArrayStamped()
        diag.header.stamp = stamp
        diag.data = [
            float(self.theta_v),
            float(self.theta_dot_v),
            float(self.theta_ddot_v),
            float(self.tau_ext_theta),
            float(tau_in),
            float(tau_spring),
            float(tau_damper),
            float(tau_wall),
            float(theta_eq),
            float(M),
            float(D),
            float(K),
        ]
        self.pub_diag.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
