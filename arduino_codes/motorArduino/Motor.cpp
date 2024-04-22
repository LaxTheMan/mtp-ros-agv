#include "Arduino.h"
#include "Motor.h"
#include <util/atomic.h>

Motor::Motor(int PWM_PIN, int DIR_PIN, int ENCA_PIN, int ENCB_PIN) {
  pwm_pin = PWM_PIN;
  dir_pin = DIR_PIN;
  enca_pin = ENCA_PIN;
  encb_pin = ENCB_PIN;

  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(enca_pin, INPUT);
  pinMode(encb_pin, INPUT);

  pos_i = 0;
  prevPos = 0;
  prevTime = 0;
  vFilt = 0.0;
  vPrev = 0.0;
}

void Motor::init() {
}

void Motor::update() {

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i;
  }

  long currTime = micros();
  float deltaTime = ((float)(currTime - prevTime)) / 1.0e6;
  float velocityRaw = (pos - prevPos) / deltaTime;
  prevPos = pos;
  prevTime = currTime;

  velocityCPS = velocityRaw;
  velocityRPM = filterVelocity(velocityRaw);
}

float Motor::filterVelocity(float raw) {
  // conversion from CPS to RPM
  float v = raw * 0.006643;
  // filtering raw velocity data
  vFilt = 0.854 * vFilt + 0.0728 * v + 0.0728 * vPrev;
  vPrev = v;

  return vFilt;
}

float Motor::getVelocityCPS() {
  return velocityCPS;
}

float Motor::getVelocityRPM() {
  return velocityRPM;
}

int Motor::getCounts() {
  return pos_i;
}

void Motor::setPWM(int pwm, int dir) {
  if (dir == 1) {
    digitalWrite(dir_pin, 1);
  } else if (dir == -1) {
    digitalWrite(dir_pin, 0);
  } else {
    pwm = 0;
  }
  analogWrite(pwm_pin, pwm);
}