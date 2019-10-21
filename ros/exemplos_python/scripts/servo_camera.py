#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
from std_msgs.msg import Float32
pos = 4

if __name__=="__main__":

	rospy.init_node("move_camera")

	posicao_camera = rospy.Publisher("/servo_camera/position", Float32, queue_size = 1 )
	

	while not rospy.is_shutdown():
		print("Oeee")
		pos = pos + 0.5
		posicao_camera.publish(pos)
        if pos > 9:
            pos = 4
		rospy.sleep(2)