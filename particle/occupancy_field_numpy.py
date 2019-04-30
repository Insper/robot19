""" An implementation of an occupancy field that you can use to implement
    your particle filter's laser_update function """


from copy import deepcopy


from random import gauss

import math
import time

import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors
import cv2

class OccupancyField(object):
    """ Stores an occupancy field for an input map.  An occupancy field returns the distance to the closest
        obstacle for any coordinate in the map
        Attributes:
            map: the map to localize against (nav_msgs/OccupancyGrid)
            closest_occ: the distance for each entry in the OccupancyGrid to the closest obstacle
    """

    def __init__(self, map):
        map = 255 - cv2.cvtColor(map, cv2.COLOR_RGBA2GRAY)
        self.map = map      # save this for later
        # build up a numpy array of the coordinates of each grid cell in the map
        h, w = self.map.shape[:2]
        X = np.zeros((w*h,2))

        # while we're at it let's count the number of occupied cells
        total_occupied = 0
        curr = 0
        for i in range(w):
            for j in range(h):
                # occupancy grids are stored in row major order, if you go through this right, you might be able to use curr
                if self.map[j,i] > 0:
                    total_occupied += 1
                X[curr,0] = float(i)
                X[curr,1] = float(j)
                curr += 1

        # build up a numpy array of the coordinates of each occupied grid cell in the map
        O = np.zeros((total_occupied,2))
        curr = 0
        for i in range(w):
            for j in range(h):
                # occupancy grids are stored in row major order, if you go through this right, you might be able to use curr
                if self.map.data[j,i] > 0:
                    O[curr,0] = float(i)
                    O[curr,1] = float(j)
                    curr += 1

        # use super fast scikit learn nearest neighbor algorithm
        nbrs = NearestNeighbors(n_neighbors=1,algorithm="ball_tree").fit(O)
        distances, indices = nbrs.kneighbors(X)

        print("Using scikit-learn to compute nearest neighbors")
        self.distances = distances
        self.indices = indices

        self.closest_occ = self.map.astype(np.float32)
        curr = 0
        for i in range(w):
            for j in range(h):
                self.closest_occ[j,i] = distances[curr][0]
                curr += 1
        self.total_occupied = total_occupied

    @property
    def width(self):
        return self.map.shape[1]

    @property
    def height(self):
        return self.map.shape[0]

    def get_closest_obstacle_distance(self,x,y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in the map.  If the (x,y) coordinate
            is out of the map boundaries, nan will be returned. """
        x, y = int(x), int(y)

        # check if we are in bounds
        if x >= self.width or x < 0 or y >= self.height or y < 0:
            return float('nan')

        return self.closest_occ[y,x]


if __name__ == '__main__':
    from PIL import Image as PilImage

    pil_image = PilImage.open('sparse_obstacles.png', 'r')
    np_image = np.asarray(pil_image).astype(np.uint8)

    occupancy_field = OccupancyField(np_image)
    occ_map = cv2.cvtColor(occupancy_field.closest_occ.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    gray = cv2.cvtColor(np_image, cv2.COLOR_RGBA2GRAY)
    chann = np.zeros_like(gray)
    occ_map += cv2.merge((chann, chann, 255 - gray))
    cv2.imshow('Field', occ_map)
    cv2.waitKey(0)

