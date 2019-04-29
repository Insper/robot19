#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 26 13:16:53 2018

@author: mirwox
"""

from random import randint, choice
import time
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import math
import random


import cv2

import sys
import os
os.getcwd() 
import sys
sys.path.append(os.getcwd())

import projeto_pf

import importlib


from PIL import Image as PilImage

import inspercles


color_image = cv2.imread("sparse_obstacles.png")
pil_image = color_image
np_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)


def draw_map(robot):
    inspercles.nb_draw_map(color_image, pose=robot.pose(), robot=True)    
    
def draw_map_particles(particles, robot):
    inspercles.nb_draw_map(color_image, particles = particles, initial_position = robot.pose(), pose=robot.pose(), robot=True)

def draw_laser():
    ax = inspercles.nb_draw_map(lidar_map, robot=True, pose=pose)
    ax.imshow(color_image, alpha=0.8)

