import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from os import listdir
from os.path import isfile, join
import numpy as np
import time


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_capture')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
    
        time.sleep(10)

        self.name = "printed_ammunition03"
        self.output_dir = "/Vision_dev/src/camera_test/images/" + self.name + "/"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
       
        self.bridge = CvBridge()
        self.count = 0
        

    def image_callback(self, msg):
        # Convert ROS2 image to OpenCV image
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CVBridge conversion failed: {e}')
            return
        img = cv2.resize(img, (1280, 720))

        self.count += 1
        if self.count <= 200:
            cv2.imwrite(self.output_dir + self.name + str(self.count) + ".png", img)
            cv2.putText(img, "Got " + str(self.count) + " images", (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
        else:
                cv2.putText(img, "Done", (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
        
        cv2.imshow("MediaPipe Hands", img)
        cv2.waitKey(1)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
