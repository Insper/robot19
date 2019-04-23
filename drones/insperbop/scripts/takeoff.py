#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty

empty_msg = Empty()


v = 10  # Velocidade linear
w = 5  # Velocidade angular

vel = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)
takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
landing = rospy.Publisher('bebop/land', Empty, queue_size = 1)


if __name__ == "__main__":
    rospy.init_node("roda_exemplo")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():
            takeoff.publish(empty_msg)
            rospy.sleep(5.0)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")

