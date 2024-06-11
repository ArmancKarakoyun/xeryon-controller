#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import os
import csv

class DataPublisherNode(Node):
    def __init__(self):
        super().__init__('data_publisher_node')
        self.linear_publisher = self.create_publisher(Float64, 'linear_data', 100)
        self.rotary_publisher = self.create_publisher(Float64, 'rotary_data', 100)
        
        # CSV dosyasının yolunu ayarla
        self.csv_path = '/home/armanc/ros2_ws/src/xeryon_controller/xeryon_controller/data_node/data.csv'
        
        # Dosya var mı kontrol et
        if not os.path.isfile(self.csv_path):
            self.get_logger().error(f"CSV file not found at {self.csv_path}")
            return

        # CSV dosyasını aç
        self.csv_file = open(self.csv_path, 'r')
        self.csv_reader = csv.reader(self.csv_file)
        next(self.csv_reader)  # İlk satırı (başlık satırı) atla
        
        self.publish_timer = self.create_timer(0.01, self.publish_callback)
        self.countdown_timer = self.create_timer(1, self.countdown_callback)
        self.countdown = 5  # Başlangıçta 5 saniye geri sayım
        self.start_publishing = False  # Veri yayınlamaya başlanacak mı?

    def countdown_callback(self):
        if self.countdown > 0:
            self.get_logger().info(f"Starting data streaming in {self.countdown} seconds...")
            self.countdown -= 1
        elif self.countdown == 0:
            self.get_logger().info("Data streaming started.")
            self.countdown -= 1
            self.start_publishing = True  # Veri yayınlamaya başla
        else:
            self.countdown_timer.cancel()

    def publish_callback(self):
        if self.start_publishing:
            try:
                row = next(self.csv_reader)
                time_sec = float(row[0])
                linear_displacement = float(row[1])
                angular_displacement = float(row[2])
                
                self.publish_data(linear_displacement, angular_displacement)
            except StopIteration:
                self.get_logger().info("End of CSV file reached. Data streaming finished.")
                self.csv_file.close()
                self.publish_timer.cancel()

    def publish_data(self, linear, rotary):
        linear_msg = Float64()
        rotary_msg = Float64()

        linear_msg.data = linear
        rotary_msg.data = rotary

        self.linear_publisher.publish(linear_msg)
        self.rotary_publisher.publish(rotary_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DataPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
