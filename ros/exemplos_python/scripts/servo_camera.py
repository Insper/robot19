#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32

# Os valores de posição da camera estão entre aproximadamente 4 e 8.5.
# A posição inicial da camera é definida em pos.
pos = 4



if __name__=="__main__":

    rospy.init_node("move_camera")

    posicao_camera = rospy.Publisher("/servo_camera/position", Float32, queue_size = 1 )
    

    while not rospy.is_shutdown():
        print("teste ")
        pos = pos + 0.5
        posicao_camera.publish(pos)
        if pos > = 8.5:
            pos = 4
        rospy.sleep(2)
