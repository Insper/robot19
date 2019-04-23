#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry


topico_odom = "/bebop/odom"

x = -1
y = -1
z = -1

def recebeu_leitura(dado):
    x = dado.pose.position.x
    y = dado.pose.position.y
    z = dado.pose.position.z


if __name__=="__main__":

	rospy.init_node("print_odom")

	recebe_scan = rospy.Subscriber("topico_odom", Odometry , recebeu_leitura)



	while not rospy.is_shutdown():
		print("Oeee")
		velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
		velocidade_saida.publish(velocidade)
		rospy.sleep(2)
