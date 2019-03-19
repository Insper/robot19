#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros


def identifica_cor(frame):
	'''
	Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
	'''

	# No OpenCV, o canal H vai de 0 até 179, logo cores similares ao 
	# vermelho puro (H=0) estão entre H=-8 e H=8. 
	# Precisamos dividir o inRange em duas partes para fazer a detecção 
	# do vermelho:
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	cor_menor = np.array([0, 50, 50])
	cor_maior = np.array([8, 255, 255])
	segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

	cor_menor = np.array([172, 50, 50])
	cor_maior = np.array([180, 255, 255])
	segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)


	# A operação MORPH_CLOSE fecha todos os buracos na máscara menores 
	# que um quadrado 7x7. É muito útil para juntar vários 
	# pequenos contornos muito próximos em um só.
	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

	# Encontramos os contornos na máscara e selecionamos o de maior área
	#contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	
	img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

	maior_contorno = None
	maior_contorno_area = 0

	for cnt in contornos:
	    area = cv2.contourArea(cnt)
	    if area > maior_contorno_area:
	        maior_contorno = cnt
	        maior_contorno_area = area

	# Encontramos o centro do contorno fazendo a média de todos seus pontos.
	if not maior_contorno is None :
	    cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
	    maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
	    media = maior_contorno.mean(axis=0)
	    media = media.astype(np.int32)
	    cv2.circle(frame, tuple(media), 5, [0, 255, 0])
	else:
	    media = (0, 0)

	# Representa a area e o centro do maior contorno no frame
	font = cv2.FONT_HERSHEY_COMPLEX_SMALL
	cv2.putText(frame,"{:d} {:d}".format(*media),(20,100), 1, 4,(255,255,255),2,cv2.LINE_AA)
	cv2.putText(frame,"{:0.1f}".format(maior_contorno_area),(20,50), 1, 4,(255,255,255),2,cv2.LINE_AA)

	cv2.imshow('video', frame)
	cv2.imshow('seg', segmentado_cor)
	cv2.waitKey(1)

	centro = (frame.shape[0]//2, frame.shape[1]//2)

	return media, centro, maior_contorno_area