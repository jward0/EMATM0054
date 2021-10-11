#include "linesensor.h"
#include "motors.h"

#define NB_LS_PINS 3

#define DN2_THRESHOLD 2500
#define DN3_THRESHOLD 2500
#define DN4_THRESHOLD 2500

#define K_P_TURN 50
#define K_P_DRIVE 30

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

  // drive_until_line();

}


void loop() {
  
  line_sensors.readAllSensors(ls_read_times);

  float e_line;
  e_line = line_sensors.calculateLineError(ls_read_times);

  float turn_pwm;
  turn_pwm = e_line * K_P_TURN;

  float drive_pwm;
  drive_pwm = (1 - abs(e_line)) * K_P_DRIVE;

  left_drive = drive_pwm - turn_pwm;
  right_drive = drive_pwm + turn_pwm;

  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);

  delay(50);

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
