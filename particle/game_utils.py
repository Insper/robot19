# -*- coding: utf-8 -*-

import numpy as np
import cv2
import inspercles
import pygame
import math
from config import LASER_COLOR

MAZE_THICK = 2

def random_color():
    h = np.random.randint(low=0,high=180) # It's easier to randomize colors in the H component
    s = 200
    v = 255
    hsv_img = np.array([[[h,s,v]]], dtype=np.dtype('u1')) # We create a one-pixel image in HSV
    #print(hsv_img)
    rgb_img = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2RGB)
    color_val = rgb_img[0][0].astype(int)
    #color_val/=255
    #print(color_val)
    return tuple(color_val)

def draw_maze(surface, lines=inspercles.lines):
    for l in lines:
        sp = (int(l[0]), int(l[1])) 
        ep = (int(l[2]), int(l[3])) 
        color = random_color()
        pygame.draw.line(surface, color, sp, ep, MAZE_THICK)

def draw_laser_readings(surface, particle, leituras):
    for a in leituras.keys():
        l = leituras[a]
        origin = (particle.x, particle.y)
        dx = l*math.cos(a + particle.theta) 
        dy = l*math.sin(a + particle.theta)
        endpoint = (int(particle.x + dx), int(particle.y + dy))
        pygame.draw.line(surface, LASER_COLOR, origin, endpoint,  3)

    
def main():
    rc = random_color()
    print(rc)    


if __name__ == "__main__":
    main()

