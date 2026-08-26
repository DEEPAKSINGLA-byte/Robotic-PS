import numpy as np
import json
from local_object import LocalObject
from tracker import Tracker

class MapManager:
    def __init__(self):
        self.objects = []
        self.next_object_id = 0
        self.tracker = Tracker()
        self.log_file = "tracking_logs.json"
        
        with open(self.log_file, "w") as f:
            json.dump([], f)

    def _append_log(self, log_entry):
        try:
            with open(self.log_file, "r") as f:
                logs = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            logs = []
            
        logs.append(log_entry)
        
        with open(self.log_file, "w") as f:
            json.dump(logs, f, indent=4)
    def merge_duplicates(self):
        spatial_threshold = 0.2
        clip_threshold = 0.85
        i = 0
        while i < len(self.objects):
            j = i + 1
            while j < len(self.objects):
                obj1 = self.objects[i]
                obj2 = self.objects[j]
                
                if obj1.class_name != obj2.class_name:
                    j += 1
                    continue
                    
                dist = self.point_cloud_distance(obj1.points, obj2.points)
                if dist > spatial_threshold:
                    j += 1
                    continue
                    
                clip_score = self.tracker.cosine_similarity(obj1.feature, obj2.feature)
                if clip_score < clip_threshold:
                    j += 1
                    continue
                    
                # Merge obj2 into obj1
                total_observations = obj1.observations + obj2.observations
                merged_feature = (obj1.feature * obj1.observations + obj2.feature * obj2.observations) / total_observations
                
                # Re-normalize
                if hasattr(merged_feature, 'norm'):
                    merged_feature = merged_feature / merged_feature.norm(dim=-1, keepdim=True)
                else:
                    merged_feature = merged_feature / np.linalg.norm(merged_feature)
                
                # Update obj1
                obj1.points = np.vstack([obj1.points, obj2.points])
                obj1.feature = merged_feature
                obj1.observations = total_observations
                
                # Pop obj2, do NOT increment j
                self.objects.pop(j)
            i += 1

    def point_cloud_distance(self,points1,points2):
        min1=np.min(points1,axis=0)
        max1=np.max(points1,axis=0)
        min2=np.min(points2,axis=0)
        max2=np.max(points2,axis=0)
        
        gap=np.maximum(min2-max1,min1-max2)
        gap = np.maximum(gap, 0)
        return np.linalg.norm(gap)
        
    def save_map(self, filepath="final_map.json"):
        final_objects = []
        for obj in self.objects:
            final_objects.append({
                "object_id": obj.id,
                "class_name": obj.class_name,
                "points_count": len(obj.points),
                "observations": getattr(obj, 'observations', 1)
            })
        with open(filepath, "w") as f:
            json.dump(final_objects, f, indent=4)
            

    def process_observation(self, class_name, points, feature):
        log_entry = {
            "observation": {
                "class_name": class_name,
                "points_count": len(points),
                "feature_shape": list(feature.shape)
            }
        }
        
        matched_object = self.tracker.find_match(class_name, points, feature, self.objects)
        
        if matched_object:
            log_entry["action"] = "matched_existing"
            log_entry["object_id"] = matched_object.id
            log_entry["object_class"] = matched_object.class_name
            
            matched_object.add_observation(points, feature)
            self._append_log(log_entry)
            
            return matched_object
        else:
            log_entry["action"] = "created_new"
            log_entry["object_id"] = self.next_object_id
            log_entry["object_class"] = class_name
            
            new_object = LocalObject(self.next_object_id, class_name, points, feature)
            self.objects.append(new_object)
            self.next_object_id += 1
            
            self._append_log(log_entry)
            
            return new_object