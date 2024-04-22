#!/usr/bin/env python3
import rospy
from simple_pid import PID
from std_msgs.msg import Int32

pid = PID(0.5, 30, 0, setpoint=0)
pid.output_limits = (0, 255)

target_angular_velocity = Int32()
target_angular_velocity.data = 50  # target velocity in rpm

actual_angular_velocity = Int32()


def velocity_callback(data):
    global actual_angular_velocity
    actual_angular_velocity = data


def update_pid(target_angular_velocity, actual_angular_velocity):
    global pid

    pwm_value = Int32()

    pid.setpoint = target_angular_velocity.data
    pwm_value.data = pid(actual_angular_velocity.data)

    return pwm_value


if __name__ == "__main__":
    try:
        rospy.init_node("controller_node", anonymous=True)
        pub = rospy.Publisher("pwm_value", Int32, queue_size=10)
        rospy.Subscriber("actual_angular_velocity", Int32, velocity_callback)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            pwm_value = update_pid(
                target_angular_velocity, actual_angular_velocity)
            rospy.loginfo(actual_angular_velocity.data)

            pub.publish(pwm_value)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
