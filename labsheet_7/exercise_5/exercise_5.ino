#define PID_UPDATE 20

#include "encoders.h"
#include "motors.h"
#include "pid.h"

Motor_c motors;
PID_c left_controller;
PID_c right_controller;

float left_rotational_velocity = 0;
float right_rotational_velocity = 0;

long last_e_left;
long last_e_right;
float last_v_left = 0;
float last_v_right = 0;

float left_drive = 0;
float right_drive = 0;

unsigned long pid_ts;
unsigned long elapsed_t;

unsigned long large_timer;
unsigned long large_timer_start;

float left_demand;
float right_demand;

void setup() {

  left_controller.initialise(0.005, 0.001, 0.1);
  right_controller.initialise(0.005, 0.001, 0.1);
  
  setupEncoderRight();
  setupEncoderLeft();
  Serial.begin(9600);
  delay(5000);

  left_demand = 2400;
  right_demand = 2400;
  pid_ts = millis();
  large_timer_start = millis();
}

void loop() {
  
  unsigned long current_ts;
  current_ts = millis();

  long pid_check = current_ts - pid_ts;

  if(pid_check > PID_UPDATE) {

    last_v_left = calculate_left_wheel_velocity(pid_check, last_v_left);
    left_drive += left_controller.update(left_demand, last_v_left, pid_check);

    last_v_right = calculate_right_wheel_velocity(pid_check, last_v_right);
    right_drive += right_controller.update(right_demand, last_v_right, pid_check);

    motors.setMotorPower("left", left_drive);
    motors.setMotorPower("right", right_drive);

    Serial.println(left_drive);
    Serial.println(right_drive);

    pid_ts = millis();
    
  }

  large_timer = millis();
  
  if(large_timer - large_timer_start > 1000) {
    left_demand *= -1;
    right_demand *= -1;
    large_timer_start = millis();
  }

}


float calculate_left_wheel_velocity(unsigned long ts, float left_last_value) {

  long instantaneous_e_left = count_e_left;
  long delta_e_left = instantaneous_e_left - last_e_left;
  float alpha = 0.2;
  last_e_left = instantaneous_e_left;
    
  left_rotational_velocity = (1 - alpha) * ((float)delta_e_left / ((float)ts/1000)) + alpha * left_last_value;

  // Serial.println(left_rotational_velocity, 4);
  return left_rotational_velocity;
}

float calculate_right_wheel_velocity(unsigned long ts, float right_last_value) {

  long instantaneous_e_right = count_e_right;
  long delta_e_right = instantaneous_e_right - last_e_right;
  float alpha = 0.2;
  last_e_right = instantaneous_e_right;
    
  right_rotational_velocity = (1 - alpha) * ((float)delta_e_right / ((float)ts/1000)) + alpha * right_last_value;

  // Serial.println(left_rotational_velocity, 4);
  return right_rotational_velocity;
}
