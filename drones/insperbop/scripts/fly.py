#! /usr/bin/env python
# -*- coding:utf-8 -*-

"""

Faz um drone que ja decolou voar em frente

"""

import rospy

from geometry_msgs.msg import Twist, Vector3

from std_msgs.msg import Empty

empty_msg = Empty()

w = 0  # Velocidade angular





if __name__ == "__main__":
    rospy.init_node("fly")
    pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1, latch=True)
    landing = rospy.Publisher('bebop/land', Empty, queue_size = 1, latch=True)
    zerov = Twist(Vector3(0,0,0), Vector3(0,0,0))

    v = 0.3  # Velocidade linear
    vel = Twist(Vector3(v,0,0), Vector3(0,0,0))

    count = 15

    try:
        while count > 0:
            pub.publish(vel)
            rospy.sleep(0.3)
            count -= 1
            print(count)
        pub.publish(zerov)
        rospy.sleep(2.0)
    except rospy.ROSInterruptException:
        pub.publish(vel)
        rospy.sleep(1.0)




