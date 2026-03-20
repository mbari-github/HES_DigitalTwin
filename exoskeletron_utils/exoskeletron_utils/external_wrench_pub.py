#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class ExternalWrenchSinePublisher(Node):
    def __init__(self):
        super().__init__('external_wrench_pub')

        # Parametri ROS2
        self.declare_parameter('topic_name', '/exo_dynamics/external_wrench')
        self.declare_parameter('publish_rate', 200.0)   # Hz
        self.declare_parameter('frequency', 0.05)        # Hz della sinusoide
        self.declare_parameter('f_min', -12.0)           # N
        self.declare_parameter('f_max', 0.0)            # N
        self.declare_parameter('frame_id', 'world')

        topic_name = str(self.get_parameter('topic_name').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.publisher_ = self.create_publisher(WrenchStamped, topic_name, 10)

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f'Publishing sinusoidal wrench on {topic_name} '
            f'with rate={publish_rate} Hz'
        )

    def timer_callback(self):
        freq = float(self.get_parameter('frequency').value)
        f_min = float(self.get_parameter('f_min').value)
        f_max = float(self.get_parameter('f_max').value)
        frame_id = str(self.get_parameter('frame_id').value)

        # Tempo corrente [s]
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        # Parametri della sinusoide
        amp = 0.5 * (f_max - f_min)
        offset = 0.5 * (f_max + f_min)

        # Sinusoide tra f_min e f_max, che parte da 0 N
        # con f_min=-7 e f_max=0 -> varia tra 0 e -7
        f = offset + amp * math.sin(2.0 * math.pi * freq * t + math.pi / 2.0)

        msg = WrenchStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id

        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = f

        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExternalWrenchSinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()