import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from mobile_sam import sam_model_registry, SamPredictor
import numpy as np
import cv2
from ultralytics import FastSAM


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )
        self.mask_pub = self.create_publisher(Image, '/fused_segmentation', 10)

        self.bridge = CvBridge()

        device = 'cuda' if torch.cuda.is_available() else 'cpu'

        self.fastsam = FastSAM("FastSAM-s.pt")
        self.fastsam.to(device)

        self.model = YOLO("yolov8x-worldv2.pt")
        self.model.to(device)

        self.sam = sam_model_registry["vit_t"](checkpoint="mobile_sam.pt")
        self.sam.to(device=device)
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
    def calculate_iou(self, box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])

        intersection_width = max(0, x2 - x1)
        intersection_height = max(0, y2 - y1)

        intersection_area = intersection_width * intersection_height

        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])

        union_area = area1 + area2 - intersection_area

        if union_area == 0:
            return 0

        return intersection_area / union_area
    def color_similarity(self, image, box1, box2):
        x1, y1, x2, y2 = map(int, box1[:4])
        crop1=image[y1:y2,x1:x2]
        x1, y1, x2, y2 = map(int, box2[:4])
        crop2 = image[y1:y2, x1:x2]
        if crop1.size==0 or crop2.size==0:
            return 0
        hist1=cv2.calcHist([crop1],[0,1,2],None,[8,8,8],[0,256,0,256,0,256])
        hist2=cv2.calcHist([crop2],[0,1,2],None,[8,8,8],[0,256,0,256,0,256])
        cv2.normalize(hist1,hist1)
        cv2.normalize(hist2,hist2)
        sim=cv2.compareHist(hist1,hist2,cv2.HISTCMP_CORREL)
        return sim
    def merge_boxes(self,image,boxes,iou_threshold=0.5,color_threshold=0.7):
        merged=True
        while merged:
            new_boxes=[]
            used=[False]*len(boxes)
            merged=False
            for i in range(len(boxes)):
                if used[i]:
                    continue
                current_box=boxes[i]
                used[i] = True
                for j in range(i+1,len(boxes)):
                    if used[j]:
                        continue
                    other_box=boxes[j]
                    if current_box[4] != other_box[4]:
                        continue
                    iou=self.calculate_iou(current_box,other_box)
                    if iou>iou_threshold:
                        similarity=self.color_similarity(image,current_box,other_box)
                        if similarity>color_threshold:
                            best_conf = max(current_box[5], other_box[5])
                            current_box=[min(current_box[0],other_box[0]),min(current_box[1],other_box[1]),max(current_box[2],other_box[2]),max(current_box[3],other_box[3]), current_box[4], best_conf]
                            used[j]=True
                            merged=True
                new_boxes.append(current_box)
            boxes=new_boxes
        return boxes
    def calculate_mask_iou(self, mask1, mask2):
        intersection = np.logical_and(mask1, mask2).sum()
        union = np.logical_or(mask1, mask2).sum()

        if union == 0:
            return 0.0

        return intersection / union

    def fuse_masks(self, mobile_sam_masks, fastsam_masks, threshold=0.5):
        final_masks = list(mobile_sam_masks)
        
        combined_mobile_mask = None
        if mobile_sam_masks:
            combined_mobile_mask = np.any([m['mask'] for m in mobile_sam_masks], axis=0)

        for fast_mask_dict in fastsam_masks:
            fast_mask = fast_mask_dict['mask']
            keep_fast_mask = True
            for mobile_mask_dict in mobile_sam_masks:
                iou = self.calculate_mask_iou(fast_mask, mobile_mask_dict['mask'])
                if iou > threshold:
                    keep_fast_mask = False
                    break

            if keep_fast_mask:
                if combined_mobile_mask is not None:
                    fast_mask = fast_mask & ~combined_mobile_mask
                
                if fast_mask.sum() > 100:
                    fast_mask_dict['mask'] = fast_mask
                    final_masks.append(fast_mask_dict)

        return final_masks

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)
        
        boxes = []
        for box in results[0].boxes:
            box_xyxy = box.xyxy[0].cpu().numpy()
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            boxes.append([
                float(box_xyxy[0]),
                float(box_xyxy[1]),
                float(box_xyxy[2]),
                float(box_xyxy[3]),
                class_id,
                confidence
            ])

        merged_boxes = self.merge_boxes(frame, boxes, iou_threshold=0.5, color_threshold=0.7)

        mobile_sam_masks = []
        if len(merged_boxes) > 0:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.predictor.set_image(rgb_frame)

            for box_info in merged_boxes:
                box_xyxy = box_info[:4]
                class_id = box_info[4]
                confidence = box_info[5]
                masks, scores, logits = self.predictor.predict(
                    box=np.array(box_xyxy),
                    multimask_output=False
                )
                mask = masks[0]
                if mask.shape != frame.shape[:2]:
                    mask = cv2.resize(mask.astype(np.uint8), (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
                mobile_sam_masks.append({
                    'mask': mask,
                    'class_id': class_id,
                    'confidence': confidence
                })

        fastsam_masks = []
        total_mask_area = 0
        if mobile_sam_masks:
            combined_mobile_mask = np.any([m['mask'] for m in mobile_sam_masks], axis=0)
            total_mask_area = combined_mobile_mask.sum()
            
        image_area = frame.shape[0] * frame.shape[1]
        
        if len(merged_boxes) < 5 and (total_mask_area / image_area) < 0.5:
            fastsam_results = self.fastsam(
                frame,
                retina_masks=True,
                imgsz=1024,
                conf=0.2,
                iou=0.9,
                verbose=False
            )

            if fastsam_results[0].masks is not None:
                for mask in fastsam_results[0].masks.data:
                    mask = mask.cpu().numpy().astype(bool)
                    if mask.shape != frame.shape[:2]:
                        mask = cv2.resize(mask.astype(np.uint8), (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
                    fastsam_masks.append({
                        'mask': mask,
                        'class_id': -1,
                        'confidence': 0.0
                    })

        final_masks = self.fuse_masks(
            mobile_sam_masks,
            fastsam_masks,
            threshold=0.5
        )

        visualization = frame.copy()
        
        np.random.seed(42)
        class_colors = {i: tuple(np.random.randint(0, 255, 3).tolist()) for i in range(-1, 100)}
        
        for mask_dict in final_masks:
            mask = mask_dict['mask']
            class_id = mask_dict['class_id']
            conf = mask_dict['confidence']
            
            color = class_colors.get(class_id, (0, 255, 0))
            if class_id == -1:
                color = (0, 0, 255)
                
            overlay = visualization.copy()
            overlay[mask] = color
            visualization = cv2.addWeighted(
                visualization,
                0.7,
                overlay,
                0.3,
                0
            )
            
            y, x = np.where(mask)
            if len(y) > 0:
                cy, cx = int(np.mean(y)), int(np.mean(x))
                if class_id != -1 and hasattr(self.model, 'names'):
                    label_name = self.model.names.get(class_id, f"class_{class_id}")
                    label = f"{label_name} ({conf:.2f})"
                else:
                    label = "Unknown"
                cv2.putText(visualization, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Fused Segmentation", visualization)
        cv2.waitKey(1)
        
        vis_msg = self.bridge.cv2_to_imgmsg(visualization)
        vis_msg.encoding = "bgr8"
        vis_msg.header = msg.header
        self.mask_pub.publish(vis_msg)


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
