#!/usr/bin/env python3

from ultralytics import YOLO
import os
import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('/Vision_dev/src/yolo_ros/yolov8_obb/scripts/best.pt')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.camera_callback,
            10)
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img, conf = 0.50, verbose=False)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        for r in results:
            if(r.obb is not None):
                boxes = r.obb
                for box in boxes:
                    self.inference_result = InferenceResult()
                    b = box.xyxyxyxy[0].to('cpu').detach().numpy().copy() 
                    c = box.cls
                    self.inference_result.class_name = self.model.names[int(c)]
                    a = b.reshape(1,8)
                    self.inference_result.coordinates = copy.copy(a[0].tolist())

                    center_x = (a[0][0] + a[0][2] + a[0][4] + a[0][6]) / 4
                    center_y = (a[0][1] + a[0][3] + a[0][5] + a[0][7]) / 4
                    center_pixels = [int(round(center_x)), int(round(center_y))]

                    self.inference_result.center = center_pixels
                    self.yolov8_inference.yolov8_inference.append(self.inference_result)
            else:
                camera_subscriber.get_logger().info(f"no_results")

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
