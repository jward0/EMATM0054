#include "linesensor.h"
#include "motors.h"

#define DN2_THRESHOLD 2500
#define DN3_THRESHOLD 2500
#define DN4_THRESHOLD 2500

#define BUTTON_B 30
#define YELLOW_LED 13

#define NB_LS_PINS 3

#define K_P_TURN 50
#define K_P_DRIVE 30

Motor_c motors;
lineSensors_c line_sensors;

unsigned long ls_read_times[NB_LS_PINS];
float calibrated_read_times[NB_LS_PINS];
unsigned long ls_offsets[NB_LS_PINS];
float ls_scales[NB_LS_PINS];

int left_drive = 0;
int right_drive = 0;


void setup() {

  Serial.begin(9600);
  delay(1000);
  
  line_sensors.initialise();
  motors.initialise();

  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);

  calibrate_sensors();

  // drive_until_line();

}


void loop() {
  
  line_sensors.readAllSensors(ls_read_times);
  // line_sensors.applyCalibration(ls_read_times);

  float e_line;
  e_line = line_sensors.calculateLineError(ls_read_times);

  Serial.println(e_line, 5);
  line_sensors.applyCalibration(ls_read_times, calibrated_read_times);
  for (int i = 0; i < NB_LS_PINS; i ++) {
    Serial.println(calibrated_read_times[i], 5);
    calibrated_read_times[i] = max(abs(calibrated_read_times[i]), 0.0001);
  }
  e_line = line_sensors.calculateUnweightedFloatLineError(calibrated_read_times);
  Serial.println(e_line, 5);
  Serial.println("----------------------------");

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


void calibrate_sensors() {

  Serial.println("Entering calibration routine");

  unsigned long white_read_times[NB_LS_PINS];
  unsigned long black_read_times[NB_LS_PINS];

  pinMode(!BUTTON_B, INPUT);
  pinMode(YELLOW_LED, OUTPUT);
  
  line_sensors.readAllSensors(white_read_times);

  digitalWrite(YELLOW_LED, HIGH);
  delay(200);
  digitalWrite(YELLOW_LED, LOW);

  delay(5000);
  
  line_sensors.readAllSensors(black_read_times);

  for(int i = 0; i < NB_LS_PINS; i++) {

    ls_offsets[i] = white_read_times[i];
    ls_scales[i] = 1.0/(black_read_times[i] - white_read_times[i]);
    Serial.println(white_read_times[i]);
    Serial.println(black_read_times[i]);
    Serial.println(ls_offsets[i]);
    Serial.println(ls_scales[i], 5);
  }

  line_sensors.setCalibration(ls_offsets, ls_scales);

  Serial.println(line_sensors.offsets[0]);
  Serial.println(line_sensors.scales[0], 4);

  digitalWrite(YELLOW_LED, HIGH);
  delay(200);
  digitalWrite(YELLOW_LED, LOW);

  delay(5000);
}
