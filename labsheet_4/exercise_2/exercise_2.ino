#include "linesensor.h"
#include "motors.h"

#define NB_LS_PINS 3

#define DN2_THRESHOLD 2000
#define DN3_THRESHOLD 2000
#define DN4_THRESHOLD 2000

Motor_c motors;
lineSensors_c line_sensors;

unsigned long ls_read_times[NB_LS_PINS];

int left_drive = 0;
int right_drive = 0;


void setup() {
  
  line_sensors.initialise();
  motors.initialise();

  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);

  drive_until_line();

}


void loop() {
  
  line_sensors.readAllSensors(ls_read_times);

  Serial.println(ls_read_times[0]);
  Serial.println(ls_read_times[1]);
  Serial.println(ls_read_times[2]);

  if (ls_read_times[0] > DN2_THRESHOLD) {
    // turn left
    left_drive = 0;
    right_drive = 30;
    
  } else if (ls_read_times[2] > DN4_THRESHOLD) {
    // turn right
    left_drive = 30;
    right_drive = 0;
    
  } else if (ls_read_times[1] > DN3_THRESHOLD) {
    // move forwards
    left_drive = 30;
    right_drive = 30;
    
  } else {
    // move forwards 
    left_drive = 30;
    right_drive = 30;
  }

  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);

  delay(20);

}


bool is_on_line() {

  line_sensors.readAllSensors(ls_read_times);

  if (ls_read_times[0] > DN2_THRESHOLD || ls_read_times[1] > DN3_THRESHOLD || ls_read_times[2] > DN4_THRESHOLD) {
    
    return true;
  } else {

    return false;
  }
}


void drive_until_line() {

  motors.setMotorPower("left", 60);
  motors.setMotorPower("right", 60);

  while(!is_on_line) {
    
    delay(10);
  }

  motors.setMotorPower("left", 0);
  motors.setMotorPower("right", 0);
  
}
