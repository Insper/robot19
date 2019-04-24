#! /usr/bin/env python
# -*- coding:utf-8 -*-

"""
Faz o drone decolar

"""

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty

empty_msg = Empty()

vel = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)
takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1, latch=True)
landing = rospy.Publisher('bebop/land', Empty, queue_size = 1, latch=True)


if __name__ == "__main__":
    rospy.init_node("takeoff")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        rospy.sleep(3.0)
        takeoff.publish(empty_msg)
        rospy.sleep(5.0)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")

