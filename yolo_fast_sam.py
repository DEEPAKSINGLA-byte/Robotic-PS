import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from mobile_sam import sam_model_registry, SamPredictor
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

        self.sam = sam_model_registry["vit_t"](checkpoint="mobile_sam.pt")
        self.predictor = SamPredictor(self.sam)
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

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        results = self.model(frame)


        visualization = frame.copy()
        if len(results[0].boxes) > 0:
            rgb_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
            )
            self.predictor.set_image(rgb_frame)
            for box in results[0].boxes:

                box_xyxy = box.xyxy[0].cpu().numpy()

                masks, scores, logits = self.predictor.predict(
                    box=box_xyxy,
                    multimask_output=False
                )

                mask = masks[0]


                overlay = visualization.copy()
                overlay[mask] = (0, 255, 0)

                visualization = cv2.addWeighted(
                    visualization,
                    0.7,
                    overlay,
                    0.3,
                    0
                )
                x1, y1, x2, y2 = box_xyxy.astype(int)

                cv2.rectangle(
                    visualization,
                    (x1, y1),
                    (x2, y2),
                    (255, 0, 0),
                    2
                )
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = self.model.names[class_id]

                label = f"{class_name} {confidence:.2f}"

                cv2.putText(
                    visualization,
                    label,
                    (x1, max(y1 - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 0),
                    2
                )

        cv2.imshow(
            "YOLO + MobileSAM",
            visualization
        )

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
