#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "kinematics.h"
#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)


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
unsigned long global_timer;
unsigned long global_timer_start;

float heading_demand = -2*3.1415926;
float drive_demand = 30;

void setup() {

  left_controller.initialise(0.02, 0.8, 0.05);
  right_controller.initialise(0.02, 0.8, 0.05);
  //heading_controller.initialise(10., 200.0, 18000.0);
  heading_controller.initialise(50., 50., 1000.);
  motors.initialise();
  
  setupEncoderRight();
  setupEncoderLeft();
  last_e_right = count_e_right;
  last_e_left = count_e_left;
  Serial.begin(9600);
  delay(3000);
  if( SERIAL_ACTIVE )Serial.println("***RESET***");

  pid_ts = millis();
  large_timer_start = millis();
  global_timer_start = millis();
}

void loop() {
  
  unsigned long current_ts;
  current_ts = millis();

  long pid_check = current_ts - pid_ts;
  long kinematics_check = current_ts - kinematics_ts;

  update_kinematics();
  float current_theta = robot_kinematics.theta;

  Serial.println(current_theta);
  Serial.println(heading_demand);
  Serial.println("--------------------------");

  if(abs(heading_demand - current_theta) > 0.01) {

    if(pid_check > PID_UPDATE) {
      
      // float rotational_component = heading_controller.update(heading_demand*1.02, current_theta, pid_check);
  
      //Serial.println(rotational_component);
      //Serial.println(current_theta);
      //Serial.println(heading_demand);
      //Serial.println("------------------");
  
      left_drive = left_controller.update(drive_demand, last_v_left, pid_check);
      right_drive = right_controller.update(-1*drive_demand, last_v_right, pid_check);
  
      //left_drive = left_controller.update(-1*rotational_component, last_v_left, pid_check);
      //right_drive = right_controller.update(rotational_component, last_v_right, pid_check);
      last_v_left = left_drive;
      last_v_right = right_drive;
  
      //left_drive = -1*rotational_component;
      //right_drive = rotational_component;
      
  
      motors.setMotorPower("left", left_drive);
      motors.setMotorPower("right", right_drive);
  
      pid_ts = millis();
      
    }
  }

  else {

    delay(25);

    left_drive = 0;
    right_drive = 0;

    motors.setMotorPower("left", left_drive);
    motors.setMotorPower("right", right_drive);
    
    update_kinematics();
    
    while(true) {
      
      reportResultsOverSerial();
      delay(1000);
      
    }
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

void update_kinematics() {
  
    long instantaneous_e_right = count_e_right;
    long instantaneous_e_left = count_e_left;
    long delta_e_right = instantaneous_e_right - last_e_right;
    long delta_e_left = instantaneous_e_left - last_e_left;
    last_e_right = instantaneous_e_right;
    last_e_left = instantaneous_e_left;
  
    robot_kinematics.update(delta_e_right, delta_e_left);
}

void reportResultsOverSerial() {

  // Print millis for debug so we can 
  // validate this is working in real
  // time, and not glitched somehow
  if( SERIAL_ACTIVE ) Serial.print( "Time(ms): " );
  if( SERIAL_ACTIVE ) Serial.println( millis() );
  delay(1);


  // Loop through array to print all 
  // results collected

  if( SERIAL_ACTIVE ) {
    Serial.print(robot_kinematics.x);
    Serial.print(" ,");
    Serial.print(robot_kinematics.y);
    Serial.print(" ,");
    Serial.println(robot_kinematics.theta - heading_demand, 3);
  }

  if( SERIAL_ACTIVE ) Serial.println( "---End of Results ---\n\n" ); 

}
