#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from Xeryon import Xeryon, Stage, Units

class LinearMotorController(Node):
    def __init__(self):
        super().__init__('linear_node')
        
        # Initialize the actuator controller
        self.controller = Xeryon("/dev/ttyACM2", 115200)  # Update with the correct device file
        self.axisX = self.controller.addAxis(Stage.XLA_78_3N, "X")  # Update with correct stage and axis name
        
        # Start the controller and find the initial index
        self.controller.start()
        self.axisX.findIndex()
        
        # Set units to nm
        self.axisX.setUnits(Units.nm)
        
        # Subscribe to the linear_data topic
        self.subscription = self.create_subscription(
            Float64,
            'linear_data',
            self.listener_callback,100)
        
        self.get_logger().info("Linear motor controller node has been initialized.")
        
        # Set up a timer to call the update_position function every 0.01 seconds
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.update_position)
        self.displacement = None

    def listener_callback(self, msg):
        # Extract displacement from the message
        self.displacement = msg.data

    def update_position(self):
        if self.displacement is not None:
            # Set the actuator's position
            self.axisX.setDPOS(self.displacement)
            
            # Print the received displacement
            self.get_logger().info(f"Updated displacement: {self.displacement} nm")

def main(args=None):
    rclpy.init(args=args)
    node = LinearMotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
