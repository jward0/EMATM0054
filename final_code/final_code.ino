#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "kinematics.h"
#include "linesensor.h"
#include "globals.h"

unsigned long rotation_start;
unsigned long rotation_ts;
int desired_mm;
int start_x;
int start_y;

void setup() {
  Serial.begin(9600);
  delay(5000);
  Serial.println("starting");

}

void loop() {

  current_ts = millis();

  long pid_check = current_ts - pid_ts;

  update_kinematics();

  switch(state) {
    case INITIALISATION:
      initialise_robot();
      break;

    case SEARCHING:
      left_drive = 40;
      right_drive = 40;
      motors.setMotorPower("left", left_drive);
      motors.setMotorPower("right", right_drive);
      break;

    case FOLLOWING:
      follow_line();
      break;

    case LINE_END:
      break;

    case CONTROLLED_ROTATING:
      if(pid_check > PID_UPDATE) {
        controlled_rotate(pid_check);
        pid_ts = millis();
      }
      break;

    case CONTROLLED_DRIVING:
      if(pid_check > PID_UPDATE) {
        controlled_drive(pid_check);
        pid_ts = millis();
      }
      break;

    case FINISHED:
      left_drive = 0;
      right_drive = 0; 
      motors.setMotorPower("left", left_drive);
      motors.setMotorPower("right", right_drive);
      break;

    case S_ERROR:
      break;
  }

  update_state();

}

void update_state() {

  switch(state) {
    case INITIALISATION:
      if(robot_initialised) {
        state = SEARCHING;
      }
      break;

    case SEARCHING:
      if(on_line()) {
        left_drive = 0;
        right_drive = 0;
        motors.setMotorPower("left", left_drive);
        motors.setMotorPower("right", right_drive);
        if(not traversing_break) {
          rotate_right();
        } else {
          break_traversed = true;
        }
        state = FOLLOWING;
      }
      break;

    case FOLLOWING:
      if(not on_line()) {
        if(not break_traversed) {
          state = SEARCHING;
          traversing_break = true;
        } else {
        state = LINE_END;
        }
      }
      break;

    case LINE_END:
      heading_demand = atan2(-1*robot_kinematics.y, -1*robot_kinematics.x);
      state = CONTROLLED_ROTATING;
      rotation_start = millis();
      break;

    case CONTROLLED_ROTATING:

      if(millis() - rotation_start > 1000) {

        desired_mm = sqrt(sq(robot_kinematics.x) + sq(robot_kinematics.y));
        start_x = robot_kinematics.x;
        start_y = robot_kinematics.y;

        state = CONTROLLED_DRIVING;
        left_demand = 2400;
        right_demand = 2400;
      }
      break;

    case CONTROLLED_DRIVING:
      if (sqrt(sq(robot_kinematics.x - start_x) + sq(robot_kinematics.y - start_y)) > desired_mm) {
        
        state = FINISHED;
      }
      break;

    case FINISHED:
      break;

    case S_ERROR:
      break;
  }
}

void initialise_robot() {
  left_controller.initialise(0.002, 0.001, 0.05);
  right_controller.initialise(0.002, 0.001, 0.05);
  heading_controller.initialise(50.0, 10.0, 1000.0);
  motors.initialise();
  setupEncoderRight();
  setupEncoderLeft();
  last_e_right = count_e_right;
  last_e_left = count_e_left;
  line_sensors.initialise();
  calibrate_sensors();
  robot_initialised = true;

  pid_ts = millis();
  large_timer_start = millis();
}

bool on_line() {
  
  line_sensors.readAllSensors(ls_read_times);
  Serial.println(ls_read_times[0]);
  Serial.println(ls_read_times[1]);
  Serial.println(ls_read_times[2]);

  if (ls_read_times[0] > DN2_THRESHOLD || ls_read_times[1] > DN3_THRESHOLD || ls_read_times[2] > DN4_THRESHOLD) {
    Serial.println("on line!");
    return true;
  } else {
    Serial.println("not on line.");
    return false;
  }
}

void rotate_right() {
  left_drive = 60;
  right_drive = 60;
  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);
  delay(120);
  left_drive = 60;
  right_drive = -60;
  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);
  while(not on_line()) {
    delay(5);
  }
  delay(50);
  left_drive = 0;
  right_drive = 0;
  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);
  
}

void follow_line() {
  
  line_sensors.readAllSensors(ls_read_times);
  // line_sensors.applyCalibration(ls_read_times);

  float e_line;
  e_line = line_sensors.calculateLineError(ls_read_times);

  line_sensors.applyCalibration(ls_read_times, calibrated_read_times);
  for (int i = 0; i < NB_LS_PINS; i ++) {
    Serial.println(calibrated_read_times[i], 5);
    calibrated_read_times[i] = max(abs(calibrated_read_times[i]), 0.0001);
  }
  e_line = line_sensors.calculateUnweightedFloatLineError(calibrated_read_times);

  float turn_pwm;
  turn_pwm = e_line * K_P_TURN;

  float drive_pwm;
  drive_pwm = (1 - abs(e_line)) * K_P_DRIVE;

  left_drive = drive_pwm - turn_pwm;
  right_drive = drive_pwm + turn_pwm;

  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);

}

void controlled_rotate(long pid_check) {
  
  float current_theta = robot_kinematics.theta;

  float rotational_component = heading_controller.update(heading_demand, current_theta, pid_check);

  right_drive = rotational_component;
  left_drive = -1*rotational_component;

  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);
}

void controlled_drive(long pid_check) {

  last_v_left = calculate_left_wheel_velocity(pid_check, last_v_left);
  left_drive += left_controller.update(left_demand, last_v_left, pid_check);

  last_v_right = calculate_right_wheel_velocity(pid_check, last_v_right);
  right_drive += right_controller.update(right_demand, last_v_right, pid_check);

  motors.setMotorPower("left", left_drive);
  motors.setMotorPower("right", right_drive);
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
