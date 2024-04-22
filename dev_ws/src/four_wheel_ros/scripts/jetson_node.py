#!/usr/bin/env python3
import rospy

from four_wheel_ros.msg import IMUState, DriverMotorState, ServoState

target_angular_velocity = DriverMotorState()
servo_angles = ServoState()
curr_imu = IMUState()
desired_imu = ()
curr_position = (5,6)
desired_position = (5,6)

def imu_callback(data):
    global curr_imu

    curr_imu = data


def position_callback(data):
    global curr_imu
    # get desired position
    # calculate current position from imu state


def robot_velocity_from_position(curr_imu, desired_imu):
    pass


def wheel_velocity_from_robot_velocity(linear_velocity, angular_velocity):
    
    
    
    pass


def calculate_robot_state():
    global curr_imu, desired_imu
    # do high level control calculation to get linear velocity, angular velocity from curr and desired positions

    linear_velocity, angular_velocity = robot_velocity_from_position(
        curr_imu, desired_imu)

    target_angular_velocity, servo_angles = wheel_velocity_from_robot_velocity(
        linear_velocity, angular_velocity)

    # do low level control to get wheel angular velocity and module steering velocity from linear and angular velocity
    return target_angular_velocity, servo_angles


if __name__ == '__main__':
    try:
        rospy.init_node('jetson_node', anonymous=True)
        rospy.Subscriber("current_imu",
                         IMUState, imu_callback)
        rospy.Subscriber("desired_position",
                         DriverMotorState, position_callback)
        pub1 = rospy.Publisher("target_angular_velocity",
                               DriverMotorState, queue_size=10)
        pub2 = rospy.Publisher("target_steer_angle", ServoState, queue_size=10)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            target_angular_velocity, servo_angles = calculate_robot_state()

            pub1.publish(target_angular_velocity)
            pub2.publish(servo_angles)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass