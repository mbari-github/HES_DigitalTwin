#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tkinter as tk


class SimpleTorqueGUI(Node):
    def __init__(self):
        super().__init__('simple_torque_gui')
        
        # Publisher
        self.publisher = self.create_publisher(Float64, '/exo_dynamics/external_wrench', 10)
        self.current_torque = 0.0
        
        # Crea GUI
        self.root = tk.Tk()
        self.root.title("Torque")
        self.root.geometry("300x200")
        
        # Etichetta coppia attuale
        self.label = tk.Label(self.root, text="T_in: 0.0 Nm", font=("Arial", 14))
        self.label.pack(pady=20)
        
        # Slider
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
        
        # Pulsante zero
        tk.Button(self.root, text="ZERO", command=self.set_zero).pack(pady=10)
        
        self.get_logger().info("Nodo torque GUI avviato")

    def slider_changed(self, value):
        """Quando lo slider viene mosso"""
        torque = float(value)
        self.current_torque = torque
        self.label.config(text=f"T_in: {torque:.1f} Nm")
        
        # Pubblica sul topic
        msg = Float64()
        msg.data = torque
        self.publisher.publish(msg)

    def set_zero(self):
        """Imposta coppia a zero"""
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