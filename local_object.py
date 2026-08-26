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

        self.observations += 1
