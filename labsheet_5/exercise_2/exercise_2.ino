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

enum state_enum {
  INITIAL,
  DRIVE_FORWARDS,
  FOUND_LINE,
  STOPPED
};

state_enum state;

int left_drive = 0;
int right_drive = 0;

void setup() {
  
  Serial.begin(9600);
  state = INITIAL;

  delay(1000);
}

void loop() {

  switch (state) {
    case INITIAL:
      initialise_robot();
      break;
      
    case DRIVE_FORWARDS:
      drive_forwards();
      break;
      
    case FOUND_LINE:
      found_line();
      break;

    case STOPPED:
      Serial.println("Stopped!");
      break;
    
    default:
      Serial.print("Error: Unknown state ");
      Serial.println(state);
      stop_robot();
      break;
  }
  
  update_state();
}

void update_state() {

  switch(state) {

    case INITIAL:
      Serial.println("State changing to DRIVE_FORWARDS");
      state = DRIVE_FORWARDS;
      break;

    case DRIVE_FORWARDS:
      if(is_on_line()) {
        
        left_drive = 0;
        right_drive = 0;
        motors.setMotorPower("left", left_drive);
        motors.setMotorPower("right", right_drive);

        Serial.println("State changing to FOUND_LINE");
        state = FOUND_LINE;
      } else {
        
      }
      break;
      
    case FOUND_LINE:
      stop_robot();
      break;

    case STOPPED:
      stop_robot();
      break;

    default:
      Serial.println("foo");
      Serial.print("Error: Unknown state ");
      Serial.println(state);
      stop_robot();
      break;      
  }
}

bool is_on_line() {
  
  line_sensors.readAllSensors(ls_read_times);
  for (int i = 0; i < NB_LS_PINS; i++) {
    if (ls_read_times[i] > DN2_THRESHOLD) {
      return true;
    }
  }

  return false;
}

void drive_forwards() {

  left_drive = 60;
  right_drive = 60;
  
  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);
}

void initialise_robot() {
  
  line_sensors.initialise();
  motors.initialise();

  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);

  calibrate_sensors();
}

void found_line() {
  for(int i = 0; i < 5; i++) {
    
    digitalWrite(YELLOW_LED, HIGH);
    delay(500);
    digitalWrite(YELLOW_LED, LOW);
    delay(500);
  }

}

void stop_robot() {
  
  left_drive = 0;
  right_drive = 0;
  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);
        
  state = STOPPED;
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
