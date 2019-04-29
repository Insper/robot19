# -*- coding: utf-8 -*-
#!/usr/bin/env python

""" Classes do filtro de particulas

    Adaptado do código de Olin/Paul Ruvolo. Source: https://github.com/paulruvolo/robot_localization_2017/blob/master/my_localizer/scripts/pf.py 

"""

from copy import deepcopy
from random import gauss
import math
import time
import numpy as np
from numpy.random import random_sample


class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """ 

    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    # TODO: define additional helper functions if needed
    def normalize(self, Z):
        """ Ajusta o peso da particula usando o fator de normalizacao (Z)
            Útil quando vamos fazer a probabilidade de todas as partículas somar 1
            """
        self.w /= Z
        
    def pose(self):
        """
            Retorna uma lista com o x, y, theta da particula
        """
        return [self.x, self.y, self.theta]
    

    def fields(self):
        """
            Retorna uma lista com x,y, theta, w da particula
        """
        l = self.pose(self)
        l.append(self.w)
        return l
    
    def set_pose(self,pose):
        self.x = pose[0]
        self.y = pose[1]
        self.theta = pose[2]


    
        


def create_particles(pose, var_x = 50, var_y = 50, var_theta = math.pi/3, num=10):
    """
        Cria num particulas
        uniformemente situadas no intervalo x - var_x a x + var_x, y - var_x at'e y + var_y e theta - var_theta a theta + var_theta
        retorna uma lista de objetos Particle
    """
    particle_cloud = []
    s = pose
    for i in range(num):
        x = np.random.uniform(s[0] - var_x, s[0] + var_x)
        y = np.random.uniform(s[1] - var_x, s[1] + var_y)
        theta = np.random.uniform(s[2] - var_theta, s[2] + var_theta)
        p = Particle(x, y, theta, w=1.0) # A prob. w vai ser normalizada depois
        particle_cloud.append(p)
    return particle_cloud


def draw_random_sample(choices, probabilities, n):
    """ Return a random sample of n elements from the set choices with the specified probabilities
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
        n: the number of samples
    """
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(n), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples


