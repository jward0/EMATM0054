#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "kinematics.h"
#include "irsensor.h"
#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

#define PID_UPDATE 10

Kinematics_c robot_kinematics;

long last_e_left;
long last_e_right;
float last_v_left = 0;
float last_v_right = 0;

float left_drive = 0;
float right_drive = 0;

unsigned long kinematics_ts;
unsigned long pid_ts;
unsigned long elapsed_t;

float heading_demand = 2*3.1415926;
float drive_demand = 30;

void setup() {
  //left_controller.initialise(0.02, 0.8, 0.05);
  //right_controller.initialise(0.02, 0.8, 0.05);
  //motors.initialise();
  
  setupEncoderRight();
  setupEncoderLeft();
  last_e_right = count_e_right;
  last_e_left = count_e_left;
  Serial.begin(9600);
  delay(5000);
  if( SERIAL_ACTIVE )Serial.println("***RESET***");

  pid_ts = millis();
  
}

void loop() {

  //unsigned long current_ts;
  //current_ts = millis();

  //long pid_check = current_ts - pid_ts;
  
  update_kinematics();
  Serial.println(robot_kinematics.theta, 3);
  delay(100);
  //float current_theta = robot_kinematics.theta; 

  //if(abs(heading_demand - current_theta) > 0.01) {

    //if(pid_check > PID_UPDATE) {

      //left_drive = left_controller.update(-1*drive_demand, last_v_left, pid_check);
      //right_drive = right_controller.update(drive_demand, last_v_right, pid_check);
  
      //last_v_left = left_drive;
      //last_v_right = right_drive;
    
      //motors.setMotorPower("left", left_drive);
      //motors.setMotorPower("right", right_drive);
  
      //pid_ts = millis();

      //if(current_theta == 10) {
        
        //ir_sensors.readAllSensors(elapsed_times)
      //}
    
    //}

  //}

  //else {

    //delay(25);

    //left_drive = 0;
    //right_drive = 0;

    //motors.setMotorPower("left", left_drive);
    //motors.setMotorPower("right", right_drive);
    
    //update_kinematics();
    
    //while(true) {
      
      //reportResultsOverSerial();
      //delay(1000);
      
    //}
  //}

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
