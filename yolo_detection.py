import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        self.model = YOLO("yolov8x-worldv2.pt")

        self.model.set_classes([
            "sofa",
            "bed",
            "chair",
            "table",
            "television",
            "refrigerator",
            "trash bin",
            "door"
        ])

    def image_callback(self, msg):

    # 1. ROS Image → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. Run YOLO
        results = self.model(frame)

        # 3. Draw detections on the image
        annotated_frame = results[0].plot()

        # 4. Display the annotated image
        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()