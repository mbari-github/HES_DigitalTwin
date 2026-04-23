#!/usr/bin/env python3
"""
Publishes a sinusoidal torque profile directly as a scalar on
/exo_dynamics/tau_ext_theta_override (std_msgs/Float64).

The signal oscillates between tau_min and tau_max:

    tau(t) = offset + amp * sin(2π f t + π/2)

where offset = (tau_max + tau_min)/2, amp = (tau_max - tau_min)/2.
The +π/2 phase makes the signal start at tau_max (i.e. zero force)
and ramp toward tau_min (peak flexion).

ROS 2 Parameters
-----------------
  topic_name      (str,   default '/exo_dynamics/tau_ext_theta_override')
  publish_rate    (float, default 200.0)   [Hz]
  frequency       (float, default 0.05)    [Hz]
  tau_min         (float, default -0.8)    [Nm]  peak flexion torque
  tau_max         (float, default  0.0)    [Nm]  zero torque
"""

import math
import rclpy
from rclpy.node import Node
from exoskeleton_safety_msgs.msg import Float64Stamped


class TauExtSinePublisher(Node):

    def __init__(self):
        super().__init__('tau_ext_sine_pub')

        self.declare_parameter('topic_name',
                               '/exo_dynamics/tau_ext_theta_override')
        self.declare_parameter('publish_rate', 200.0)
        self.declare_parameter('frequency', 0.05)
        self.declare_parameter('tau_min', -0.8)
        self.declare_parameter('tau_max', 0.0)

        topic_name   = str(self.get_parameter('topic_name').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.publisher_ = self.create_publisher(Float64Stamped, topic_name, 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f'Sine torque publisher on {topic_name} @ {publish_rate} Hz | '
            f'freq={self.get_parameter("frequency").value} Hz, '
            f'tau=[{self.get_parameter("tau_min").value}, '
            f'{self.get_parameter("tau_max").value}] Nm'
        )

    def timer_callback(self):
        freq    = float(self.get_parameter('frequency').value)
        tau_min = float(self.get_parameter('tau_min').value)
        tau_max = float(self.get_parameter('tau_max').value)

        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        amp    = 0.5 * (tau_max - tau_min)
        offset = 0.5 * (tau_max + tau_min)
        tau = offset + amp * math.sin(2.0 * math.pi * freq * t + math.pi / 2.0)

        msg = Float64Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = tau
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TauExtSinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()