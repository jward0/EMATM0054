#include "encoders.h"
#include "kinematics.h"
#include "motors.h"

Motor_c motors;
Kinematics_c robot_kinematics;
long last_e_right;
long last_e_left;

int left_drive = 0;
int right_drive = 0;

void setup() {
  setupEncoderRight();
  setupEncoderLeft();
  last_e_right = count_e_right;
  last_e_left = count_e_left;
  Serial.begin(9600);
  delay(1000);

}

void loop() {

  while(robot_kinematics.theta < 1.57) {

    left_drive = -40;
    right_drive = 40;
    motors.setMotorPower("left", left_drive);
    motors.setMotorPower("right", right_drive);

    delay(10);

    long instantaneous_e_right = count_e_right;
    long instantaneous_e_left = count_e_left;
    long delta_e_right = instantaneous_e_right - last_e_right;
    long delta_e_left = instantaneous_e_left - last_e_left;
    last_e_right = instantaneous_e_right;
    last_e_left = instantaneous_e_left;
  
    robot_kinematics.update(delta_e_right, delta_e_left);
   
  }

  left_drive = 0;
  right_drive = 0;
  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);  

  delay(500);

  while(robot_kinematics.theta > 0) {

    left_drive = 40;
    right_drive = -40;
    motors.setMotorPower("left", left_drive);
    motors.setMotorPower("right", right_drive);

    delay(10);

    long instantaneous_e_right = count_e_right;
    long instantaneous_e_left = count_e_left;
    long delta_e_right = instantaneous_e_right - last_e_right;
    long delta_e_left = instantaneous_e_left - last_e_left;
    last_e_right = instantaneous_e_right;
    last_e_left = instantaneous_e_left;
  
    robot_kinematics.update(delta_e_right, delta_e_left);
   
  }

  left_drive = 0;
  right_drive = 0;
  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);  

  delay(500);

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
