#!/usr/bin/env python3
import rospy

from std_msgs.msg import Header, String, Int32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy

vel_msg = Twist()

def callback_controller(data):
    global vel_msg
    
    # vel_msg.linear.x = data.axes[1]
    # vel_msg.linear.y = data.axes[0]
    # vel_msg.angular.z = data.axes[3]
    vel_msg = data

def commander():
    global vel_msg

    pub = rospy.Publisher('/four_wheel_steering_controller/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(50)
    print(f"vel_x is {vel_msg.linear.x}")
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        # rate.sleep()

if __name__ == '__main__':
    print("In main")
    rospy.init_node('commander', anonymous=True)
    # rospy.Subscriber("joy0", Joy, callback_controller)
    rospy.Subscriber("cmd_vel", Twist, callback_controller)
    commander()
    rospy.spin()