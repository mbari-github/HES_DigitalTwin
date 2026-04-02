#!/usr/bin/env python3
"""
Tkinter GUI node for manual WrenchStamped input.

Publishes on the same topic and with the same message type as
external_wrench_pub.py (/exo_dynamics/external_wrench, WrenchStamped),
so it can be used as a drop-in interactive replacement for the automated
sine publisher during manual testing.

The slider controls force.z [N]; all other wrench components are zero.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import tkinter as tk


class SimpleTorqueGUI(Node):
    def __init__(self):
        super().__init__('simple_torque_gui')

        # Same topic and message type as external_wrench_pub.py
        self.publisher = self.create_publisher(
            WrenchStamped, '/exo_dynamics/external_wrench', 10
        )
        self.current_force = 0.0

        # Build the Tkinter window
        self.root = tk.Tk()
        self.root.title("External Wrench")
        self.root.geometry("350x250")

        # Current force readout label
        self.label = tk.Label(self.root, text="Force Z: 0.0 N", font=("Arial", 14))
        self.label.pack(pady=20)

        # Slider: range ±25 N, consistent with external_wrench_pub.py defaults
        self.scale = tk.Scale(
            self.root,
            from_=-25.0,
            to=25.0,
            resolution=0.1,
            orient=tk.HORIZONTAL,
            length=300,
            command=self.slider_changed,
            label="Force Z [N]"
        )
        self.scale.set(0.0)
        self.scale.pack(pady=20)

        # Zero button
        tk.Button(self.root, text="ZERO", command=self.set_zero).pack(pady=10)

        self.get_logger().info("External Wrench GUI node started")

    def slider_changed(self, value):
        """Publish a WrenchStamped with force.z set to the slider value."""
        force = float(value)
        self.current_force = force
        self.label.config(text=f"Force Z: {force:.1f} N")

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = force

        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0

        self.publisher.publish(msg)

    def set_zero(self):
        """Reset slider to zero and publish zero force."""
        self.scale.set(0.0)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTorqueGUI()

    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()