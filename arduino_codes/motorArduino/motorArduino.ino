#include "Motor.h"
#include <ros.h>
#include <four_wheel_ros/DriverMotorState.h>
#include <four_wheel_ros/MotorPWM.h>

#define MOTOR_FL_PWM_PIN 1
#define MOTOR_FL_DIR_PIN 2
#define MOTOR_FL_ENCA_PIN 3
#define MOTOR_FL_ENCB_PIN 4

#define MOTOR_FR_PWM_PIN 5
#define MOTOR_FR_DIR_PIN 6
#define MOTOR_FR_ENCA_PIN 7
#define MOTOR_FR_ENCB_PIN 8

#define MOTOR_RL_PWM_PIN 9
#define MOTOR_RL_DIR_PIN 10
#define MOTOR_RL_ENCA_PIN 11
#define MOTOR_RL_ENCB_PIN 12

#define MOTOR_RR_PWM_PIN 13
#define MOTOR_RR_DIR_PIN 14
#define MOTOR_RR_ENCA_PIN 15
#define MOTOR_RR_ENCB_PIN 16

Motor motorFL = Motor(MOTOR_FL_PWM_PIN, MOTOR_FL_DIR_PIN, MOTOR_FL_ENCA_PIN, MOTOR_FL_ENCB_PIN);
Motor motorFR = Motor(MOTOR_FR_PWM_PIN, MOTOR_FR_DIR_PIN, MOTOR_FR_ENCA_PIN, MOTOR_FR_ENCB_PIN);
Motor motorRL = Motor(MOTOR_RL_PWM_PIN, MOTOR_RL_DIR_PIN, MOTOR_RL_ENCA_PIN, MOTOR_RL_ENCB_PIN);
Motor motorRR = Motor(MOTOR_RR_PWM_PIN, MOTOR_RR_DIR_PIN, MOTOR_RR_ENCA_PIN, MOTOR_RR_ENCB_PIN);

unsigned long currTime = 0;
unsigned long prevTime = 0;

const int LOOPTIME = 50;

float motorFLSpeed = 0;
float motorFRSpeed = 0;
float motorRLSpeed = 0;
float motorRRSpeed = 0;

void pwm_cb(const four_wheel_ros::MotorPWM motorPWM_msg) {
  motorFL.setPWM((int)fabs(motorPWM_msg.pwmFL), (motorPWM_msg.pwmFL < 0 ? -1 : 1));
  motorFR.setPWM((int)fabs(motorPWM_msg.pwmFR), (motorPWM_msg.pwmFR < 0 ? -1 : 1));
  motorRL.setPWM((int)fabs(motorPWM_msg.pwmRL), (motorPWM_msg.pwmRL < 0 ? -1 : 1));
  motorRR.setPWM((int)fabs(motorPWM_msg.pwmRR), (motorPWM_msg.pwmRR < 0 ? -1 : 1));
}

ros::NodeHandle nh;
four_wheel_ros::DriverMotorState speed_msg;
ros::Publisher speed_pub("actual_angular_velocity", &speed_msg);
ros::Subscriber<four_wheel_ros::MotorPWM> pwm_sub("motor_pwm_values", pwm_cb);

void setup() {
  nh.initNode();
  nh.advertise(speed_pub);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENCA_PIN), readEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENCA_PIN), readEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENCA_PIN), readEncoderRL, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENCA_PIN), readEncoderRR, RISING);
}

void loop() {
  currTime = millis();
  if (currTime - prevTime >= LOOPTIME) {
    prevTime = currTime;

    motorFL.update();
    motorFR.update();
    motorRL.update();
    motorRR.update();

    motorFLSpeed = motorFL.getVelocityRPM();
    motorFRSpeed = motorFR.getCounts();
    Serial.println(motorFRSpeed);
    motorRLSpeed = motorRL.getVelocityRPM();
    motorRRSpeed = motorRR.getVelocityRPM();

  }

  publishSpeed();
  nh.spinOnce();
}

void publishSpeed() {
  speed_msg.motorFLRPM = motorFLSpeed;
  speed_msg.motorFRRPM = motorFRSpeed;
  speed_msg.motorRLRPM = motorRLSpeed;
  speed_msg.motorRRRPM = motorRRSpeed;
  speed_pub.publish(&speed_msg);
}

void readEncoderFL() {
  int b = digitalRead(MOTOR_FL_ENCB_PIN);
  if (b > 0) {
    motorFL.pos_i++;
  } else {
    motorFL.pos_i--;
  }
}

void readEncoderFR() {
  int b = digitalRead(MOTOR_FR_ENCB_PIN);
  if (b > 0) {
    motorFR.pos_i++;
  } else {
    motorFR.pos_i--;
  }
}

void readEncoderRL() {
  int b = digitalRead(MOTOR_RL_ENCB_PIN);
  if (b > 0) {
    motorRL.pos_i++;
  } else {
    motorRL.pos_i--;
  }
}

void readEncoderRR() {
  int b = digitalRead(MOTOR_RR_ENCB_PIN);
  if (b > 0) {
    motorRR.pos_i++;
  } else {
    motorRR.pos_i--;
  }
}