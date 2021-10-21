#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "kinematics.h"

#define PID_UPDATE 10
#define KINEMATICS_UPDATE 10

Kinematics_c robot_kinematics;
Motor_c motors;
PID_c left_controller;
PID_c right_controller;
PID_c heading_controller;

float left_rotational_velocity = 0;
float right_rotational_velocity = 0;

long last_e_left;
long last_e_right;
float last_v_left = 0;
float last_v_right = 0;

float left_drive = 0;
float right_drive = 0;

unsigned long kinematics_ts;
unsigned long pid_ts;
unsigned long elapsed_t;

unsigned long large_timer;
unsigned long large_timer_start;

float heading_demand = atan2(1, 1);

void setup() {

  left_controller.initialise(0.005, 0.001, 0.1);
  right_controller.initialise(0.005, 0.001, 0.1);
  heading_controller.initialise(50.0, 10.0, 1000.0);
  motors.initialise();
  
  setupEncoderRight();
  setupEncoderLeft();
  last_e_right = count_e_right;
  last_e_left = count_e_left;
  Serial.begin(9600);
  delay(5000);

  pid_ts = millis();
  large_timer_start = millis();
}

void loop() {
  
  unsigned long current_ts;
  current_ts = millis();

  long pid_check = current_ts - pid_ts;
  long kinematics_check = current_ts - kinematics_ts;

  update_kinematics();

  if(pid_check > PID_UPDATE) {
    
    float current_theta = robot_kinematics.theta;

    float rotational_component = heading_controller.update(heading_demand, current_theta, pid_check);

    Serial.println(rotational_component);
    Serial.println(current_theta);
    Serial.println(heading_demand);
    Serial.println("------------------");

    right_drive = rotational_component;
    left_drive = -1*rotational_component;

    motors.setMotorPower("left", left_drive);
    motors.setMotorPower("right", right_drive);

    pid_ts = millis();
    
  }

  large_timer = millis();

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

void update_kinematics() {
  
    long instantaneous_e_right = count_e_right;
    long instantaneous_e_left = count_e_left;
    long delta_e_right = instantaneous_e_right - last_e_right;
    long delta_e_left = instantaneous_e_left - last_e_left;
    last_e_right = instantaneous_e_right;
    last_e_left = instantaneous_e_left;
  
    robot_kinematics.update(delta_e_right, delta_e_left);
}
