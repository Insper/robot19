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
import sys


class Particle(object):
    """ Representa uma hipótese sobre a posição do robô consistindo de x,y and theta
        Atributos:
            x: coordenada x no sistema de coordenadas do mapa
            y: coordenada y no sistema de coordenadas do mapa
            theta: ângulo do robô em relação ao sistema de coordenadas do mapa
            w: P(H_i) para a partícula
    """ 

    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        """ Constrói uma nova partícula
            x: coordenada x no sistema de coordenadas do mapa
            y: coordenada y no sistema de coordenadas do mapa
            theta: ângulo do robô em relação ao sistema de coordenadas do mapa
            w: P(H_i) para a partícula. A normalização (fazer somar 1) não é feita
            automaticamente
            """
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

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
    
    def x_y(self):
        """
            Retorna apenas as componentes x, y da pose
        """
        return (self.x, self.y)
    

    def pose_prob(self):
        """
            Retorna uma lista com x,y, theta, w da particula
        """
        l = self.pose()
        l.append(self.w)
        return l
    
    def set_pose(self,pose):
        """
            Inicializa x, y e theta com uma lista de 3 numeros recebida
        """
        self.x = pose[0]
        self.y = pose[1]
        self.theta = pose[2]
        
    def set_pose_prob(self, pose_prob):
        """
            Recebe uma lista com [x,y,theta, w] e guarda estes valores
        """
        self.set_pose(self, pose_prob[:-1])
        self.w = pose_prob[-1]

    def __getitem__(self, index):
        """
            Permite que as particulas sejam acessadas como uma lista de 4 valores
        """
        
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        elif index==2:
            return self.theta
        elif index==3:
            return self.w
                        
    def __setitem__(self, index, value):
        """
            Permite a sintaxe p[numero] = valor para cada particula, 
            em que 0,1,2,3 são x,y,theta e w, respectivamente
        """
        if index == 0:
            self.x = value
        elif index == 1:
            self.y = value
        elif index==2:
            self.theta = value
        elif index==3:
            self.w = value
            
    def move(self, movimento):
        """
            Desloca a particula recebendo um array de movimento x, y, theta
        """
        self.x += movimento[0]
        self.y += movimento[1]
        self.theta += movimento[2]
        
    def move_linear(self, desl):
        """
            Realiza um deslocamento relativo de magnitude desl no sentido
            Em que o robô está olhando
        """
        self.x = self.x + math.cos(self.theta)*desl
        self.y = self.y + math.sin(self.theta)*desl # Edit- (-) sign for pygame
        
    def move_relative(self, speed):
        """
            speed[0] is a linear speed
            speed[1] is an angular speed
        """
        self.move_angular(speed[1])        
        self.move_linear(speed[0])
        
  
    def move_angular(self, ang):
        """
            Realiza um deslocamento angular de magnitude ang
        """
        self.theta+=ang

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
    """ 
        Devolve uma amostra aleatória de n elementos retirada do conjunto choices em que cada 
        elemento tem uma probabilidade diferente de ser escolhido. As probabilidades
        estão na lista probabilities
        
        choices: lista de valores a amostrar
        probabilities: lista das probabilidades de cada valor
        n: número de amostras desejadas na lista resultado
    """
    if np.any(np.isnan(probabilities)):
        message = """\n\nIMPOSSÍVEL calcular com valor de probabilidade NaN - Not a number presentes na lista \n
        DICA: se estiver obtendo probabilidades muito pequenas \n 
        Cheque se suas contas estão certas, lembre-se de que a produtória \n
        da fórmula se aplica apenas aos lasers de cada partícula \n
        Caso precise mesmo representar valores menores que  {0} \n
        Use mpmath.mpf para armazenar os valores temporários do cálculo de probabilidade e de alpha \n
        e volte a armazenar no atributo w da particula após a multiplicação por alpha \n
        referênia: https://docs.sympy.org/0.6.7/modules/mpmath/basics.html"""
        message = message.format(sys.float_info.min)
        print(message)
        print("Suas probabilidades:")
        print(probabilities)
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(n), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples



