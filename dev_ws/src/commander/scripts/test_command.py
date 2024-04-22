#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def callback_controller(data):
    print(data)

if __name__ == '__main__':
    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("cmd_vel",Twist,callback_controller)
    rospy.spin()