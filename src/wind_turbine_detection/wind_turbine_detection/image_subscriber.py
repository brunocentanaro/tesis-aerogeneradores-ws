import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('wind_turbine_detection')
model_path = os.path.join(package_share_directory, 'resource', 'yolov8n.pt')
model_not_turbine_path = os.path.join(package_share_directory, 'resource', 'yolov8m.pt')

model = YOLO(model_path)
modelNotTurbine = YOLO(model_not_turbine_path)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10)
        self.subscription
        self.br = CvBridge()
        # print(model.names)
        # print(modelNotTurbine.names)
   
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        image = current_frame
        results = model.predict(image)
        print(model.names) 
        img = results[0].plot()
        cv2.imshow('Detected Frame', img)    
        cv2.waitKey(1)
  
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
