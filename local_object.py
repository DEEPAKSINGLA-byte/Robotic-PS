import numpy as np


class LocalObject:

    def __init__(self, object_id, class_name, points, feature):

        self.id = object_id
        self.class_name = class_name
        self.points = points
        self.feature = feature
        self.observations = 1

    def add_observation(self, points, feature):

        self.points = np.vstack([
            self.points,
            points
        ])

        self.feature = (self.feature * self.observations + feature) / (self.observations + 1)
        
        if hasattr(self.feature, 'norm'):
            self.feature = self.feature / self.feature.norm(dim=-1, keepdim=True)
        else: #
            norm = np.linalg.norm(self.feature)
            if norm > 0:
                self.feature = self.feature / norm

        self.observations += 1
