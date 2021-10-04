#include "motors.h"

int pwm_value;
bool pwm_value_increasing;

Motor_c motors;

void setup() {
  motors.initialise();
}

void loop() {

  motors.setMotorPower("left", pwm_value);
  motors.setMotorPower("right", pwm_value);
  delay(20);

  if (pwm_value >= 220) {
    pwm_value_increasing = false;
  }
  else if (pwm_value <= -220) {
    pwm_value_increasing = true;
  }

  if (pwm_value_increasing) {
    pwm_value += 1;
  }
  else {
    pwm_value -= 1;
  }
}
