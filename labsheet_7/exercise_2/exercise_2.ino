#define PID_UPDATE 20

#include "encoders.h"
#include "motors.h"
#include "pid.h"

Motor_c motors;
PID_c controller;

float left_rotational_velocity = 0;

long last_e_left;
float last_v_left = 0;

int left_drive = 0;
int right_drive = 0;

unsigned long pid_ts;
unsigned long elapsed_t;

void setup() {

  controller.initialise(0.02, 0, 0);
  
  setupEncoderRight();
  setupEncoderLeft();
  Serial.begin(9600);
  delay(1000);

  pid_ts = millis();
}

void loop() {
  
  unsigned long current_ts;
  current_ts = millis();

  long pid_check = current_ts - pid_ts;

  if(pid_check > PID_UPDATE) {

    last_v_left = calculate_left_wheel_velocity(pid_check, last_v_left);
    left_drive += controller.update(2400, last_v_left);

    motors.setMotorPower("left", left_drive);

    Serial.println(last_v_left, 4);
    Serial.println(left_drive);

    pid_ts = millis();
    
  }

}


float calculate_left_wheel_velocity(unsigned long ts, float last_value) {

  long instantaneous_e_left = count_e_left;
  long delta_e_left = instantaneous_e_left - last_e_left;
  float alpha = 0.2;
  last_e_left = instantaneous_e_left;
    
  left_rotational_velocity = (1 - alpha) * ((float)delta_e_left / ((float)ts/1000)) + alpha * last_value;

  Serial.println(left_rotational_velocity, 4);
  return left_rotational_velocity;
}
