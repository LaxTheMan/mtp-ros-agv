#!/usr/bin/env python3
import rospy

from four_wheel_ros.msg import DriverMotorState, MotorPWM
from simple_pid import PID

target_angular_velocity = DriverMotorState()
actual_angular_velocity = DriverMotorState()

pid1 = PID(1, 0.1, 0, setpoint=1)
pid2 = PID(1, 0.1, 0, setpoint=1)
pid3 = PID(1, 0.1, 0, setpoint=1)
pid4 = PID(1, 0.1, 0, setpoint=1)

pid1.output_limits = (0, 255)
pid2.output_limits = (0, 255)
pid3.output_limits = (0, 255)
pid4.output_limits = (0, 255)


def callback1(data):
    global target_angular_velocity
    target_angular_velocity = data


def callback2(data):
    global actual_angular_velocity
    actual_angular_velocity = data


def update_pid(target_angular_velocity, actual_angular_velocity):
    global pid1, pid2, pid3, pid4
    pwm_values = MotorPWM()

    pid1.setpoint = target_angular_velocity.motorFLRPM
    pid2.setpoint = target_angular_velocity.motorFRRPM
    pid3.setpoint = target_angular_velocity.motorRLRPM
    pid4.setpoint = target_angular_velocity.motorRRRPM

    pwm_values.pwmFL = pid1(actual_angular_velocity.motorFLRPM)
    pwm_values.pwmFR = pid2(actual_angular_velocity.motorFRRPM)
    pwm_values.pwmRL = pid3(actual_angular_velocity.motorRLRPM)
    pwm_values.pwmRR = pid4(actual_angular_velocity.motorRRRPM)

    return pwm_values


if __name__ == '__main__':
    try:
        rospy.init_node('pid_controller', anonymous=True)
        rospy.Subscriber("target_angular_velocity",
                         DriverMotorState, callback1)
        rospy.Subscriber("actual_angular_velocity",
                         DriverMotorState, callback2)
        pub = rospy.Publisher("servo_pwm_values", MotorPWM, queue_size=10)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            pwm_values = update_pid(
                target_angular_velocity, actual_angular_velocity)
            rospy.loginfo(pwm_values)
            pub.publish(pwm_values)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
