#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Esta classe deve conter todas as suas implementações relevantes para seu filtro de partículas
"""

from pf import Particle, create_particles
import numpy as np
import math

S = [] # lista das partículas


largura = 775
altura = 748

# Os angulos em que o robo simulado vai ter sensores
angles = np.linspace(0.0, 2*math.pi, num=9)

# Robo
robot = Particle(largura/2, altura/2, math.pi/4, 1.0)

# Nuvem de particulas
particle_cloud = []


def create_random_particles(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=10):
    """
        Pedido no item 1
        Cria uma lista de partículas distribuídas de forma uniforme entre minx, miny, maxx e maxy
        
        garanta que os argumentos são passados
        
    """
    return create_particles(robot.pose())







