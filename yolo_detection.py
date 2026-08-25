import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from mobile_sam import sam_model_registry, SamPredictor
import numpy as np
import cv2
import mobileclip 
import tf2_ros
from PIL import Image as PILImage

class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')
        self.latest_depth=None
        self.fx=None
        self.fy=None
        self.cx=None
        self.cy=None
        self.depth_sub=self.create_subscription(Image,'/camera/depth_image',self.depth_callback,10)
        self.info_sub=self.create_subscription(CameraInfo,'/camera/camera_info',self.info_callback,10)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        self.clip_checkpoint = "/home/deepak/InterIIT_practice/task_2/mobileclip_s2.pt" # Update this to your MobileCLIP checkpoint path
        self.clip_model, _, self.clip_preprocess = mobileclip.create_model_and_transforms(
            "mobileclip_s2",
            pretrained=None, # Set this back to self.clip_checkpoint once you download the correct weights
            device=self.device
        )
        self.clip_model.eval()
        self.clip_tokenize = mobileclip.get_tokenizer("mobileclip_s2")

        self.bridge = CvBridge()

        self.model = YOLO("yolov8x-worldv2.pt")
        self.model.to(self.device)

        self.sam = sam_model_registry["vit_t"](checkpoint="mobile_sam.pt")
        self.sam.to(device=self.device)
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
    def depth_callback(self,msg):
        self.latest_depth=self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
    def mask_to_camera_points(self,mask):
        if self.latest_depth is None:
            return None
        if self.fx is None:
            return None
        ys,xs=np.where(mask)
        depth_value=self.latest_depth[ys,xs]
        valid=(depth_value>0) & np.isfinite(depth_value)
        xs=xs[valid]
        ys=ys[valid]
        depth_value=depth_value[valid]
        z=depth_value
        x=(xs-self.cx)*z/self.fx
        y=(ys-self.cy)*z/self.fy
        return np.stack([x,y,z],axis=1)  

    def info_callback(self,msg):
        self.fx=msg.k[0]
        self.fy=msg.k[4]
        self.cx=msg.k[2]
        self.cy=msg.k[5]
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
    def get_clip_features(self,image,box_xyxy,class_name):
        x1,y1,x2,y2=box_xyxy
        crop=image[int(y1):int(y2),int(x1):int(x2)]
        pil_image=PILImage.fromarray(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB))
        processed_image=self.clip_preprocess(pil_image).unsqueeze(0).to(self.device)
        text=self.clip_tokenize([class_name]).to(self.device)
        with torch.no_grad():
            image_features=self.clip_model.encode_image(processed_image)
            text_features=self.clip_model.encode_text(text)
            image_features/=image_features.norm(dim=-1,keepdim=True)
            text_features/=text_features.norm(dim=-1,keepdim=True)
            final_feature=0.7*image_features+0.3*text_features
            final_feature=final_feature/final_feature.norm(dim=-1,keepdim=True)
            return final_feature
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
                    iou=self.calculate_iou(current_box,other_box)
                    if iou>iou_threshold:
                        similarity=self.color_similarity(image,current_box,other_box)
                        if similarity>color_threshold:
                            best_class_id = other_box[4] if other_box[5] > current_box[5] else current_box[4]
                            best_conf = max(current_box[5], other_box[5])
                            current_box=[min(current_box[0],other_box[0]),min(current_box[1],other_box[1]),max(current_box[2],other_box[2]),max(current_box[3],other_box[3]), best_class_id, best_conf]
                            used[j]=True
                            merged=True
                new_boxes.append(current_box)
            boxes=new_boxes
        return boxes
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

        visualization = frame.copy()
        if len(merged_boxes) > 0:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.predictor.set_image(rgb_frame)

            for box_info in merged_boxes:
                box_xyxy = box_info[:4]
                class_id = int(box_info[4])
                confidence = float(box_info[5])

                
                masks, scores, logits = self.predictor.predict(
                    box=np.array(box_xyxy),
                    multimask_output=False
                )
                mask = masks[0]
                points_camera = self.mask_to_camera_points(mask)
                
                class_name = self.model.names[class_id]
                overlay = visualization.copy()
                overlay[mask] = (0, 255, 0)

                visualization = cv2.addWeighted(visualization, 0.7, overlay, 0.3, 0)
                
                x1, y1, x2, y2 = map(int, box_xyxy)
                cv2.rectangle(visualization, (x1, y1), (x2, y2), (255, 0, 0), 2)

                feature=self.get_clip_features(frame,box_xyxy,class_name)

                label = f"{class_name} {confidence:.2f}"
                cv2.putText(visualization, label, (x1, max(y1 - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.imshow("YOLO + MobileSAM", visualization)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()