#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import Joy
from four_wheel_ros.msg import ServoState, MotorPWM, DriverMotorState
from geometry_msgs.msg import Twist
from simple_pid import PID

x_vel = 0
y_vel = 0
z_ang_vel = 0

wheel_radius = 0.05
wheel_max_rpm = 112.5
gear_ratio = 1/1.5 #specify gear ratio, if not present set as 1

target_angular_velocity = {'FL': 0, 'FR': 0, 'RL': 0, 'RR': 0}
target_steer_angle = {'FL': 0, 'FR': 0, 'RL': 0, 'RR': 0}

actual_angular_velocity = DriverMotorState()

pid1 = PID(0.5, 30, 0, setpoint=128)
pid2 = PID(0.5, 30, 0, setpoint=128)
pid3 = PID(0.5, 30, 0, setpoint=128)
pid4 = PID(0.5, 30, 0, setpoint=128)

pid1.output_limits = (0, 255)
pid2.output_limits = (0, 255)
pid3.output_limits = (0, 255)
pid4.output_limits = (0, 255)


def joystick_callback(data: Joy):
    global x_vel, y_vel, z_ang_vel

    x_vel = data.axes[1]*wheel_max_rpm*math.pi*wheel_radius/30 #map linear velocity to joystick limits
    y_vel = -data.axes[0]*wheel_max_rpm*math.pi*wheel_radius/30
    z_ang_vel = -data.axes[2]*wheel_max_rpm*math.pi/30


def keyboard_callback(data: Twist):
    global x_vel, y_vel, z_ang_vel

    x_vel = (data.linear.x/100)*wheel_max_rpm*math.pi * \
        wheel_radius/30  # calculated as linear vel% of max rpm
    y_vel = -(data.linear.y/100)*wheel_max_rpm*math.pi*wheel_radius/30
    z_ang_vel = (data.angular.z/100)*wheel_max_rpm*math.pi/30


def velocity_callback(data: DriverMotorState):
    global actual_angular_velocity

    actual_angular_velocity = data


def robot_to_wheel_velocity(target_angular_velocity: dict, target_steer_angle: dict):

    # parallel wheels mode
    v = math.hypot(x_vel, y_vel) / wheel_radius
    sign_x = np.sign(x_vel)
    sign_y = np.sign(y_vel)

    if (x_vel != 0):
        ang = y_vel / x_vel
    else:
        ang = 0
    if (x_vel != 0):
        target_angular_velocity['FL'] = sign_x*v
        target_angular_velocity['FR'] = sign_x*v
        target_angular_velocity['RL'] = sign_x*v
        target_angular_velocity['RR'] = sign_x*v
    elif(y_vel != 0):
        target_angular_velocity['FL'] = sign_y*v
        target_angular_velocity['FR'] = sign_y*v
        target_angular_velocity['RL'] = sign_y*v
        target_angular_velocity['RR'] = sign_y*v
    else:
        target_angular_velocity['FL'] = 0
        target_angular_velocity['FR'] = 0
        target_angular_velocity['RL'] = 0
        target_angular_velocity['RR'] = 0
        
        

    if (x_vel == 0 and y_vel != 0):    
        target_steer_angle['FL'] = 90
        target_steer_angle['FR'] = 90
        target_steer_angle['RL'] = int(target_steer_angle['FL'])
        target_steer_angle['RR'] = int(target_steer_angle['FR'])
    else:
        target_steer_angle['FL'] = int(math.atan(ang)*180/math.pi)
        target_steer_angle['FR'] = int(math.atan(ang)*180/math.pi)
        target_steer_angle['RL'] = int(target_steer_angle['FL'])
        target_steer_angle['RR'] = int(target_steer_angle['FR'])

    return target_angular_velocity, target_steer_angle


def update_servo_angles(target_steer_angle: dict):
    servo_angles = ServoState()

    # do servo to wheel angle mapping
    
    

    servo_angles.servoFLAngle = (target_steer_angle['FL']+90)
    servo_angles.servoFRAngle = (target_steer_angle['FR']+90)
    servo_angles.servoRLAngle = (target_steer_angle['RL']+90)
    servo_angles.servoRRAngle = (target_steer_angle['RR']+90)

    return servo_angles


def update_motor_pwm(target_angular_velocity: dict, actual_angular_velocity: DriverMotorState):
    global pid1, pid2, pid3, pid4
    motor_pwm = MotorPWM()

    # wheel velocity to motor velocity
    pid1.setpoint = target_angular_velocity['FL'] * gear_ratio
    pid2.setpoint = target_angular_velocity['FR'] * gear_ratio
    pid3.setpoint = target_angular_velocity['RL'] * gear_ratio
    pid4.setpoint = target_angular_velocity['RR'] * gear_ratio

    # calculate motor pwm with pid
    # capping motor pwm at 50 for testing
    motor_pwm.pwmFL = pid1(actual_angular_velocity.motorFLRPM)
    motor_pwm.pwmFR = pid2(actual_angular_velocity.motorFRRPM)
    motor_pwm.pwmRL = pid3(actual_angular_velocity.motorRLRPM)
    motor_pwm.pwmRR = pid4(actual_angular_velocity.motorRRRPM)

    return motor_pwm


if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous=True)
        # rospy.Subscriber('joy', Joy, joystick_callback)
        rospy.Subscriber("cmd_vel", Twist, keyboard_callback)
        rospy.Subscriber("actual_angular_velocity",
                         DriverMotorState, velocity_callback)
        servo_pub = rospy.Publisher("servo_angles", ServoState, queue_size=10)
        motor_pub = rospy.Publisher(
            "motor_pwm_values", MotorPWM, queue_size=10)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            target_angular_velocity, target_steer_angle = robot_to_wheel_velocity(
                target_angular_velocity, target_steer_angle)
            servo_angles = update_servo_angles(target_steer_angle)
            motor_pwm = update_motor_pwm(
                target_angular_velocity, actual_angular_velocity)
            
            rospy.loginfo(f"Target Wheel Angular Velocity: {target_angular_velocity}")
            rospy.loginfo(f"Target Steer Angle: {target_steer_angle}")
            rospy.loginfo(f"Servo Angles: {servo_angles}")
            rospy.loginfo(f"Motor PWM Values: {motor_pwm}")

            servo_pub.publish(servo_angles)
            motor_pub.publish(motor_pwm)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
