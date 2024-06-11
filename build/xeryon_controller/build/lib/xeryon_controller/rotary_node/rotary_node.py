#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from Xeryon import Xeryon, Stage, Units

class RotaryMotorController(Node):
    def __init__(self):
        super().__init__('rotary_node')

        # Initialize the actuator controller
        try:
            self.controller = Xeryon("/dev/ttyACM0", 115200)  # Update with the correct device file
            self.axisX = self.controller.addAxis(Stage.XRTU_30_3, "X")  # Update with correct stage and axis name
        except Exception as e:
            self.get_logger().error(f"Failed to initialize controller: {e}")
            return

        # Start the controller and find the initial index
        try:
            self.controller.start()
            self.axisX.findIndex()
            # Set units to degrees
            self.axisX.setUnits(Units.deg)
        except Exception as e:
            self.get_logger().error(f"Failed to start controller or find index: {e}")
            return

        # Subscribe to the rotary_data topic
        self.subscription = self.create_subscription(
            Float64,
            'rotary_data',
            self.listener_callback,
            10)

        self.get_logger().info("Rotary motor controller node has been initialized.")

        # Set up a timer to call the update_position function every 0.01 seconds
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.update_position)
        self.displacement = None

    def listener_callback(self, msg):
        # Extract displacement from the message
        self.displacement = msg.data

    def update_position(self):
        if self.displacement is not None:
            try:
                # Set the actuator's position
                self.axisX.setDPOS(self.displacement)
                # Log the received displacement
                self.get_logger().info(f"Updated displacement: {self.displacement} deg")
            except Exception as e:
                self.get_logger().error(f"Failed to update position: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RotaryMotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
