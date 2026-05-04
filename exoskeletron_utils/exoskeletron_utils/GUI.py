#!/usr/bin/env python3
"""
Tkinter GUI node for manual torque input.

Publishes a scalar torque value on /exo_dynamics/external_wrench (Float64)
via an interactive slider. Intended for quick open-loop testing without a
WrenchStamped publisher.

Note: this node uses Float64 (not WrenchStamped). Use GUI_control.py if you
need to drive the full WrenchStamped pipeline.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tkinter as tk


class SimpleTorqueGUI(Node):
    def __init__(self):
        super().__init__('simple_torque_gui')

        self.publisher = self.create_publisher(Float64, '/exo_dynamics/external_wrench', 10)
        self.current_torque = 0.0

        # Build the Tkinter window
        self.root = tk.Tk()
        self.root.title("Torque")
        self.root.geometry("300x200")

        # Current torque readout label
        self.label = tk.Label(self.root, text="T_in: 0.0 Nm", font=("Arial", 14))
        self.label.pack(pady=20)

        # Slider: range ±5 Nm
        self.scale = tk.Scale(
            self.root,
            from_=-5,
            to=5,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            length=250,
            command=self.slider_changed
        )
        self.scale.pack(pady=20)

        # Zero button
        tk.Button(self.root, text="ZERO", command=self.set_zero).pack(pady=10)

        self.get_logger().info("Torque GUI node started")

    def slider_changed(self, value):
        """Called on every slider movement — publishes the new torque value."""
        torque = float(value)
        self.current_torque = torque
        self.label.config(text=f"T_in: {torque:.1f} Nm")

        msg = Float64()
        msg.data = torque
        self.publisher.publish(msg)

    def set_zero(self):
        """Reset slider and publish zero torque."""
        self.scale.set(0)

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