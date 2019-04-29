
# coding: utf-8

# # Filtro de partículas
# 
# 
# Cada partícula vai ser representada por uma lista [x,y,theta]
# 
# Trabalhe com 2 listas:
# 
# 	S = []  # Vai conter as n partículas. 
# 
# 	W = [] # Pesos das partículas
# 
# 	n_part = # Número de partículas
# 
# 
# # Atividades
# 
# 
# 1. Crie uma função que gere n partículas aleatórias distribuidas uniformemente dentro de uma área minx, miny, maxx, maxy (passados como parâmetros). Veja a funcao 
# 
# 	Devolve uma lista com n partículas
# 
# 2. Descubra como desenhar as n partículas (analise o módulo inspercles)
# 
# 3. Faça o desenho das partículas
# 
# 4. Crie uma lista para simular os movimentos do robô. Veja a lista chamada <code>movimentos</code> que é usada na seção *Teste de animação*, mais abaixo
# 
# 5. Crie uma função que aplica um deslocamento [delta_x, delta_y, delta_theta] com um desvio padrão [std_x, std_y, std_theta] a todas as partículas
# 
# 6. Desenhe as partículas após o deslocamento
# 
# 7. Descubra como calcular $P(D|H)$ analisando o Python Notebook. A função que traça os raios está no modulo inspercles
# 
# Programe a aplicação da equação que está na pág. 853 do livro do Norvig $$P(z_t | x_t) = \alpha \prod_{j}^M{e^{\frac{-(z_j - \hat{z_j})}{2\sigma^2}}}$$
# 
# Ou seja, para cada raio estimado $\hat{z_j}$ e leitura real $z_j$ calcule a probabilidade daquele raio ter acontecido dado que a posição do robô fosse de fato a da partícula
# 
# 
# 8 . Usando os resultados de 7 calcule $P(H|D)$ e insira numa lista de probabilidades das partículas
# 
# 9 . Reamostre as partículas de forma proporcional à probabilidade
# 
# 

# In[1]:

from random import randint, choice
import time
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import math
import random
from pf import Particle
from nav_msgs.msg import OccupancyGrid
from occupancy_field import OccupancyField
from helper_functions import angle_normalize, angle_diff

import cv2

import sys
import os
os.getcwd() 
import sys
sys.path.append(os.getcwd())


import inspercles
reload(inspercles)


# Para suprimir alguns warnings do Matplotlib Inline
import warnings
warnings.simplefilter('ignore')
#get_ipython().magic(u'matplotlib inline')


# ## Diretõrio de trabalho

# In[2]:

#get_ipython().system(u'pwd')




# In[3]:

# Dados do arquivo mapa.yaml

resolution = 0.020000
origin =  [-5.000000, -5.000000, 0.000000]
occupied_thresh = 0.8
free_thresh =  0.2

inspercles.free_thresh = free_thresh # Limiar em tons de cinza para que uma celula seja considerada livre
inspercles.occupied_thresh = occupied_thresh # Limiar de celula ocupada
inspercles.origin = origin
inspercles.resolution = resolution


# In[4]:



from PIL import Image as PilImage


color_image = cv2.imread("sparse_obstacles.png")
pil_image = color_image
np_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)


#color_image = PilImage.open("sparse_obstacles.png", 'r')
#pil_image = color_image.convert('1') # Converte a imagem PIL para Grayscale
#np_image = np.asarray(pil_image)

particle_size = 10

#import matplotlib_render as plotmap


back_color = "black"
colors     = ['red', 'green', 'cyan', 'yellow']
width, height = 775, 746

inspercles.width = width
inspercles.height = height




initial_pose = [200, 200, math.pi/8] # Posicao inicial considerada para o pf
inspercles.initial_pose = initial_pose
pose = [330, 220, math.radians(90)] # posicao "verdadeira" do robo 
inspercles.pose = pose
robot_radius=10   # Raio do robo
inspercles.robot_radius = robot_radius


# Os angulos em que o robo simulado vai ter sensores
angles = np.linspace(0.0, 2*math.pi, num=8)

particle_cloud = []


fora = Particle(x="2000", y="2000")
    
    
    
    


# ## Mapa com posição inicial

# In[5]:

inspercles.nb_draw_map(color_image, pose=pose, robot=True)


particle_cloud = inspercles.nb_initialize_particle_cloud()

particle_cloud = [fora] + particle_cloud


# A função *nb_initialize_particle_cloud()* pertence ao módulo <code>inspercles</code> e já faz uma primeira aleatorização das partículas

particulas = particle_cloud

# In[7]:

inspercles.nb_draw_map(color_image, particles = particulas, initial_position = initial_pose, pose=pose, robot=True)


inspercles.nb_draw_arrow(p.x, p.y, p.theta, ax, particle_size, color='b')


# In[8]:

angles


# # Simulação da imagem do laser

# Para simular a leitura **real** do robo, use *nb_simulate_lidar* passando a posição real do robô, os ângulos do sensor e a np_image com o mapa

# In[9]:

leituras, lidar_map = inspercles.nb_simulate_lidar(pose, angles, np_image)


# In[10]:

ax = inspercles.nb_draw_map(lidar_map, robot=True, pose=pose)
ax.imshow(color_image, alpha=0.8)
#nb_draw_map(occupancy_image)
