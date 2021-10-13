#include "encoders.h"
#include "kinematics.h"

Kinematics_c robot_kinematics;
long last_e_right;
long last_e_left;

void setup() {
  setupEncoderRight();
  setupEncoderLeft();
  last_e_right = count_e_right;
  last_e_left = count_e_left;
  Serial.begin(9600);
  delay(1000);

}

void loop() {

  long instantaneous_e_right = count_e_right;
  long instantaneous_e_left = count_e_left;
  long delta_e_right = instantaneous_e_right - last_e_right;
  long delta_e_left = instantaneous_e_left - last_e_left;
  last_e_right = instantaneous_e_right;
  last_e_left = instantaneous_e_left;
  
  robot_kinematics.update(delta_e_right, delta_e_left);
  
  Serial.println(robot_kinematics.x);
  Serial.println(robot_kinematics.y);
  Serial.println(robot_kinematics.theta);
  Serial.println("-------------------------------------");

  delay(10);
}
