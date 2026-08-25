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

        # COCO's 80 standard object classes.
        # YOLO-World also uses these as its default offline vocabulary,
        # but we define them explicitly here.
        # Curated 50-class vocabulary for common indoor/household objects.
        self.model.set_classes([
            "person",
            "chair",
            "bed",
            "nightstand",
            "table",
            "coffee table",
            "desk",
            "dining table",
            "door",
            "window",
            "cabinet",
            "kitchen cabinet",
            "refrigerator",
            "sofa",
            "trash bin",
            "tv",
            "tv cabinet",
            "vase",
            "wardrobe",
            "shoe rack",
            "air conditioner",
            "sink",
            "microwave",
            "oven",
            "toaster",
            "bottle",
            "cup",
            "bowl",
            "plate",
            "fork",
            "knife",
            "spoon",
            "book",
            "laptop",
            "computer",
            "keyboard",
            "mouse",
            "remote",
            "cell phone",
            "clock",
            "backpack",
            "handbag",
            "suitcase",
            "potted plant",
            "toilet",
            "mirror",
            "lamp",
            "fan",
            "picture frame",
            "washing machine"
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
