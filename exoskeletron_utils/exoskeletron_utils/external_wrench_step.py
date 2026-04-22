#!/usr/bin/env python3
"""
Publishes a smoothed step (ramp-and-hold) profile directly as a scalar
torque on /exo_dynamics/tau_ext_theta_override (std_msgs/Float64).

This bypasses the wrench → Jacobian → projection chain entirely,
so the admittance controller receives a TRUE step on its input
coordinate, free from the configuration-dependent distortion of
the kinematic projection B^T J^T f.

The dynamics node must have tau_ext_theta_override_enable=true
(see the companion patch) to subscribe to this topic and use the
received value in place of the internally computed tau_ext_theta.

Profile
--------
  t < t_delay                          →  0
  t_delay  ≤ t < t_delay + t_ramp     →  linear ramp from 0 to tau_step
  t_delay + t_ramp ≤ t < t_hold_end   →  hold at tau_step
  t_hold_end ≤ t < t_hold_end + t_ramp→  linear ramp back to 0
  t ≥ t_hold_end + t_ramp             →  0   (then repeat if enabled)

ROS 2 Parameters
-----------------
  topic_name      (str,   default '/exo_dynamics/tau_ext_theta_override')
  publish_rate    (float, default 200.0)   [Hz]
  tau_step        (float, default -0.6)    [Nm]  step amplitude (negative = flexion)
  t_delay         (float, default 3.0)     [s]   initial quiescent period
  t_ramp          (float, default 0.5)     [s]   ramp duration (0 = pure step)
  t_hold          (float, default 10.0)    [s]   hold duration at tau_step
  t_pause         (float, default 5.0)     [s]   pause between repetitions
  repeat          (bool,  default True)
  n_repeats       (int,   default 3)              number of repetitions (0 = inf)

Usage in launch file
---------------------
  Node(
      package='exoskeletron_utils',
      executable='tau_ext_step_pub',
      name='input',
      output='screen',
      parameters=[{
          'tau_step':  -0.6,
          't_delay':   3.0,
          't_ramp':    0.5,
          't_hold':   10.0,
          't_pause':   5.0,
          'repeat':    True,
          'n_repeats': 3,
      }],
  ),

  Remember to also pass to the dynamics node:
      'external_wrench_enable': False,
      'tau_ext_theta_override_enable': True,
"""

import rclpy
from rclpy.node import Node
from exoskeletron_safety_msgs.msg import Float64Stamped


class TauExtStepPublisher(Node):

    def __init__(self):
        super().__init__('tau_ext_step_pub')

        self.declare_parameter('topic_name',
                               '/exo_dynamics/tau_ext_theta_override')
        self.declare_parameter('publish_rate', 200.0)

        self.declare_parameter('tau_step',  -0.6)    # Nm
        self.declare_parameter('t_delay',    5.0)    # s
        self.declare_parameter('t_ramp',     0.5)    # s
        self.declare_parameter('t_hold',    20.0)    # s
        self.declare_parameter('t_pause',    5.0)    # s
        self.declare_parameter('repeat',     True)
        self.declare_parameter('n_repeats',  3)

        topic_name   = str(self.get_parameter('topic_name').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.publisher_ = self.create_publisher(Float64Stamped, topic_name, 10)

        self.t0 = self.get_clock().now()
        self._rep_count = 0
        self._finished = False

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f'tau_ext step publisher on {topic_name} @ {publish_rate} Hz | '
            f'tau_step={self.get_parameter("tau_step").value} Nm, '
            f't_ramp={self.get_parameter("t_ramp").value} s, '
            f't_hold={self.get_parameter("t_hold").value} s, '
            f'n_repeats={self.get_parameter("n_repeats").value}'
        )

    def _compute_tau(self, t: float) -> float:
        tau_step = float(self.get_parameter('tau_step').value)
        t_delay  = float(self.get_parameter('t_delay').value)
        t_ramp   = float(self.get_parameter('t_ramp').value)
        t_hold   = float(self.get_parameter('t_hold').value)
        t_pause  = float(self.get_parameter('t_pause').value)
        repeat   = bool(self.get_parameter('repeat').value)
        n_rep    = int(self.get_parameter('n_repeats').value)

        T_cycle = t_delay + 2.0 * t_ramp + t_hold + t_pause

        if repeat and (n_rep <= 0 or self._rep_count < n_rep):
            cycle = int(t // T_cycle)
            if n_rep > 0 and cycle >= n_rep:
                self._finished = True
                return 0.0
            if cycle > self._rep_count:
                self._rep_count = cycle
            t_local = t - cycle * T_cycle
        else:
            if t >= T_cycle:
                self._finished = True
                return 0.0
            t_local = t

        t1 = t_delay
        t2 = t_delay + t_ramp
        t3 = t_delay + t_ramp + t_hold
        t4 = t_delay + 2.0 * t_ramp + t_hold

        if t_local < t1:
            return 0.0
        elif t_local < t2:
            if t_ramp < 1e-9:
                return tau_step
            return tau_step * (t_local - t1) / t_ramp
        elif t_local < t3:
            return tau_step
        elif t_local < t4:
            if t_ramp < 1e-9:
                return 0.0
            return tau_step * (1.0 - (t_local - t3) / t_ramp)
        else:
            return 0.0

    def timer_callback(self):
        if self._finished:
            tau = 0.0
        else:
            now = self.get_clock().now()
            t = (now - self.t0).nanoseconds * 1e-9
            tau = self._compute_tau(t)

        msg = Float64Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = tau
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TauExtStepPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()