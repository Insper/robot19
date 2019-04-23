#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy


from geometry_msgs.msg import Twist, Vector3

v = 10  # Velocidade linear
w = 0  # Velocidade angular

pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)
takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
landing = rospy.Publisher('bebop/land', Empty, queue_size = 1)
zerov = Twist(Vector3(0,0,0), Vector3(0,0,0))
vel = Twist(Vector3(v,0,0), Vector3(0,0,w))



if __name__ == "__main__":
    rospy.init_node("fly")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    count = 5

    try:
        while count > 0:
            pub.publish(vel)
            rospy.sleep(1.0)
            count -= 1
        pub.publish(zerov)
        rospy.sleep(2.0)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")

