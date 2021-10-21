#define PID_UPDATE 20

#include "encoders.h"
#include "motors.h"
#include "pid.h"

Motor_c motors;
PID_c controller;

float left_rotational_velocity = 0;

long last_e_left;
float last_v_left = 0;

float left_drive = 0;
float right_drive = 0;

unsigned long pid_ts;
unsigned long elapsed_t;

unsigned long large_timer;
unsigned long large_timer_start;

float demand;

void setup() {

  controller.initialise(0.005, 0.001, 0.1);
  
  setupEncoderRight();
  setupEncoderLeft();
  Serial.begin(9600);
  delay(5000);

  demand = 2400;
  pid_ts = millis();
  large_timer_start = millis();
}

void loop() {
  
  unsigned long current_ts;
  current_ts = millis();

  long pid_check = current_ts - pid_ts;

  if(pid_check > PID_UPDATE) {

    last_v_left = calculate_left_wheel_velocity(pid_check, last_v_left);
    left_drive += controller.update(demand, last_v_left, pid_check);

    motors.setMotorPower("left", left_drive);

    Serial.println(left_drive);

    pid_ts = millis();
    
  }

  large_timer = millis();
  
  if(large_timer - large_timer_start > 1000) {
    demand *= -1;
    large_timer_start = millis();
  }

}


float calculate_left_wheel_velocity(unsigned long ts, float last_value) {

  long instantaneous_e_left = count_e_left;
  long delta_e_left = instantaneous_e_left - last_e_left;
  float alpha = 0.2;
  last_e_left = instantaneous_e_left;
    
  left_rotational_velocity = (1 - alpha) * ((float)delta_e_left / ((float)ts/1000)) + alpha * last_value;

  // Serial.println(left_rotational_velocity, 4);
  return left_rotational_velocity;
}
