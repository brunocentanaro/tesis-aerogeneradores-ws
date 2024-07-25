import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

windTurbineTypeAndLocation = [
    {
        "height": 50,
        "coordinates": {
            "latitude": -34.627257,
            "longitude": -54.957857
        }
    }
]

class MissionStateHandler(Node):
    def __init__(self):
        super().__init__('mission_state_handler')
        self.declare_parameter('mission_param', 0)
        mission_param = self.get_parameter('mission_param').get_parameter_value().integer_value
        self.get_logger().info(f'Mission param: {mission_param}')
        self.publisher = self.create_publisher(String, 'gps_waypoint', 10)
        time.sleep(30)
        self.get_logger().info('Publishing waypoint')
        newCoord = windTurbineTypeAndLocation[mission_param]['coordinates']
        height = windTurbineTypeAndLocation[mission_param]['height']
        self.publish_waypoint(f"{newCoord['latitude']},{newCoord['longitude']},{height}")


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