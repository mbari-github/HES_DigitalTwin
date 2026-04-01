#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import tkinter as tk


class SimpleTorqueGUI(Node):
    def __init__(self):
        super().__init__('simple_torque_gui')

        # Publisher — stesso topic e stesso tipo di external_wrench_pub.py
        self.publisher = self.create_publisher(
            WrenchStamped, '/exo_dynamics/external_wrench', 10
        )
        self.current_force = 0.0

        # Crea GUI
        self.root = tk.Tk()
        self.root.title("External Wrench")
        self.root.geometry("350x250")

        # Etichetta forza attuale
        self.label = tk.Label(self.root, text="Force Z: 0.0 N", font=("Arial", 14))
        self.label.pack(pady=20)

        # Slider forza (force.z), range coerente con external_wrench_pub.py
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

        # Pulsante zero
        tk.Button(self.root, text="ZERO", command=self.set_zero).pack(pady=10)

        self.get_logger().info("Nodo External Wrench GUI avviato")

    def slider_changed(self, value):
        """Pubblica WrenchStamped aggiornando force.z con il valore dello slider."""
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
        """Imposta la forza a zero."""
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