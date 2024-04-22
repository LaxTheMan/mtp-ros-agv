#include <QMC5883LCompass.h>
#include <Servo.h>
#include <ros.h>
#include <four_wheel_ros/IMUState.h>
#include <four_wheel_ros/ServoState.h>

#define SERVO_FL_PIN 9
#define SERVO_FR_PIN 10
#define SERVO_RL_PIN 11
#define SERVO_RR_PIN 12

QMC5883LCompass compass;

Servo servoFL;
Servo servoFR;
Servo servoRL;
Servo servoRR;

unsigned long currTime = 0;
unsigned long prevTime = 0;

const int LOOPTIME = 50;

int x = 0;
int y = 0;
int z = 0;

void servo_cb(const four_wheel_ros::ServoState &servo_msg) {
  if (servo_msg.servoFLAngle >= 0 && servo_msg.servoFLAngle <= 180) servoFL.write(servo_msg.servoFLAngle);
  if (servo_msg.servoFRAngle >= 0 && servo_msg.servoFRAngle <= 180) servoFR.write(servo_msg.servoFRAngle);
  if (servo_msg.servoRLAngle >= 0 && servo_msg.servoRLAngle <= 180) servoRL.write(servo_msg.servoRLAngle);
  if (servo_msg.servoRRAngle >= 0 && servo_msg.servoRRAngle <= 180) servoRR.write(servo_msg.servoRRAngle);
}

ros::NodeHandle nh;
four_wheel_ros::IMUState imu_msg;
ros::Publisher imu_pub("current_imu", &imu_msg);
ros::Subscriber<four_wheel_ros::ServoState> servo_sub("servo_angles", servo_cb);

void setup() {
  nh.initNode();
  nh.advertise(imu_pub);
  nh.subscribe(servo_sub);
  initServos();
  compass.init();
}

void loop() {
  currTime = millis();
  if (currTime - prevTime >= LOOPTIME) {
    prevTime = currTime;

    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();
  }
  
  publishIMU();
  nh.spinOnce();
}

void initServos() {
  servoFL.attach(SERVO_FL_PIN);
  servoFR.attach(SERVO_FR_PIN);
  servoRL.attach(SERVO_RL_PIN);
  servoRR.attach(SERVO_RR_PIN);
}

void publishIMU() {
  imu_msg.x = x;
  imu_msg.y = y;
  imu_msg.z = z; 
  imu_pub.publish(&imu_msg);
}