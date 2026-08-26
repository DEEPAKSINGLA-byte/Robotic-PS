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
        
        # Initialize an empty list in the JSON file
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