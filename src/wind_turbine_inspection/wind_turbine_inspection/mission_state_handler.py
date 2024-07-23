import rclpy
from rclpy.node import Node
from sensor_msgs.msg import String
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
import time

class MissionStateHandler(Node):
    def __init__(self):
        super().__init__('mission_state_handler')
        self.publisher = self.create_publisher(String, 'waypoint', 10)
        time.sleep(20)
        self.publish_waypoint("test")

    def publish_waypoint(self, waypoint):
        msg = String()
        msg.data = waypoint
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    mission_state_handler = MissionStateHandler()
    rclpy.spin(mission_state_handler)
    mission_state_handler.destroy_node()
    rclpy.shutdown()