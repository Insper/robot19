#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Esta classe deve conter todas as suas implementações relevantes para seu filtro de partículas
"""

from pf import Particle, create_particles, draw_random_sample
import numpy as np
import inspercles # necessário para o a função nb_lidar que simula o laser
import math
from scipy.stats import norm


largura = 775 # largura do mapa
altura = 748  # altura do mapa

# Robo
robot = Particle(largura/2, altura/2, math.pi/4, 1.0)

# Nuvem de particulas
particulas = []

num_particulas = 1000


# Os angulos em que o robo simulado vai ter sensores
angles = np.linspace(0.0, 2*math.pi, num=8, endpoint=False)

# Lista mais longa
movimentos_longos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0],
			  [0,0,math.pi/12.0], [0, 0, math.pi/12.0], [0, 0, math.pi/12],[0,0,-math.pi/4],
			  [-5, 0, 0],[-5,0,0], [-5,0,0], [-10,0,0],[-10,0,0], [-10,0,0],[-10,0,0],[-10,0,0],[-15,0,0],
			  [0,0,-math.pi/4],[0, 10, 0], [0,10,0], [0, 10, 0], [0,10,0], [0,0,math.pi/8], [0,10,0], [0,10,0], 
			  [0,10,0], [0,10,0], [0,10,0],[0,10,0],
			  [0,0,-math.radians(90)],
			  [math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],
			  [math.cos(math.pi/3)*10, math.sin(math.pi/3),0]]

# Lista curta
movimentos_curtos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0]]

movimentos_relativos = [[0, -math.pi/3],[10, 0],[10, 0], [10, 0], [10, 0],[15, 0],[15, 0],[15, 0],[0, -math.pi/2],[10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [0, -math.pi/2], 
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [0, -math.pi/2], 
					   [10,0], [0, -math.pi/4], [10,0], [10,0], [10,0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
					   [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0]]



movimentos = movimentos_relativos



def cria_particulas(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=num_particulas):
	"""
		Cria uma lista de partículas distribuídas de forma uniforme entre minx, miny, maxx e maxy
	"""
	particles_list = []

	for e in range(num_particulas):
	  x = np.random.uniform(0, largura)
	  y = np.random.uniform(0, altura)
	  theta = np.random.uniform(0, 2*math.pi)
	  particle = Particle(x, y, theta, w=1/num_particulas)

	  particles_list.append(particle)

	return particles_list
	
def move_particulas(particulas, movimento):
	"""
		Recebe um movimento na forma [deslocamento, theta]  e o aplica a todas as partículas
		Assumindo um desvio padrão para cada um dos valores
		Esta função não precisa devolver nada, e sim alterar as partículas recebidas.
		
		Sugestão: aplicar move_relative(movimento) a cada partícula
		
		Você não precisa mover o robô. O código fornecido pelos professores fará isso
		
	"""
	movimento[0] = movimento[0] + norm.rvs(loc=0, scale=0.2)
	movimento[1] = movimento[1] + norm.rvs(loc=0, scale=0.2)

	for e in particulas:
		e.move_relative(movimento)

	return particulas
	
def leituras_laser_evidencias(robot, particulas):
	"""
		Realiza leituras simuladas do laser para o robo e as particulas
		Depois incorpora a evidência calculando
		P(H|D) para todas as particulas
		Lembre-se de que a formula $P(z_t | x_t) = \alpha \prod_{j}^M{e^{\frac{-(z_j - \hat{z_j})}{2\sigma^2}}}$ 
		responde somente P(D|Hi), em que H é a hi
		
		Esta função não precisa retornar nada, mas as partículas precisa ter o seu w recalculado. 
		
		Você vai precisar calcular para o robo
		
	"""
	
	leitura_robo = inspercles.nb_lidar(robot, angles)
	scale = 10 #10cm
	alpha = 0
	# Voce vai precisar calcular a leitura para cada particula usando inspercles.nb_lidar e depois atualizar as probabilidades
	for part in particulas:
		prob = 0
		for key, value in (inspercles.nb_lidar(part, angles)).items():
			prob += norm.pdf(value, loc=leitura_robo[key], scale=scale)
		part.w = prob
		alpha += prob

	for part in particulas:
		part.normalize(alpha)

	
	
def reamostrar(particulas, n_particulas = num_particulas):
	"""
		Reamostra as partículas devolvendo novas particulas sorteadas
		de acordo com a probabilidade e deslocadas de acordo com uma variação normal    
		
		O notebook como_sortear tem dicas que podem ser úteis
		
		Depois de reamostradas todas as partículas precisam novamente ser deixadas com probabilidade igual
		
		Use 1/n ou 1, não importa desde que seja a mesma
	"""
	pesos = [p.w for p in particulas]
	particulas = draw_random_sample(particulas, pesos, num_particulas)

	for p in particulas:
	    p.w = 1/num_particulas
	    p.x += norm.rvs(loc=0, scale=10)
	    p.y += norm.rvs(loc=0, scale=10)
	    p.theta += norm.rvs(loc=0, scale=math.pi/20)

	return particulas


	





