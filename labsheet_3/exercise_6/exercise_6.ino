// Global definition of the time interval
#define LINE_SENSOR_UPDATE 100
#define MOTOR_UPDATE       2000
#define NB_LS_PINS 3
#define Y_LED 13
#include "linesensor.h"
#include "motors.h"

int pwm_value = 60;

unsigned long ls_ts;
unsigned long motor_ts;
unsigned long elapsed_t;

Motor_c motors;

lineSensors_c line_sensors;


void setup() {
  line_sensors.initialise();
  motors.initialise();

  delay(100);

  motors.setMotorPower("left", pwm_value);
  motors.setMotorPower("right", pwm_value);

  ls_ts = millis();
}


void loop() {

  // Record the time of this execution
  // of loop for coming calucations
  //  ( _ts = "time-stamp" )
  unsigned long current_ts;

  current_ts = millis();


  // Run our line sensor update
  // every 100ms (10hz).
  // Tracking time for the line sensor (ls)
  elapsed_t = current_ts - ls_ts;
  if( elapsed_t > LINE_SENSOR_UPDATE ) {

    // Conduct a read of the line sensors
    unsigned long ls_read_times[NB_LS_PINS];
    line_sensors.readAllSensors(ls_read_times);
    Serial.println(ls_read_times[1]);
    if (ls_read_times[1] > 2500) { // determine actual value to go here when testing

      digitalWrite(Y_LED, HIGH); 
    }
    else {

      digitalWrite(Y_LED, LOW);     
    }
        
    // Record when this execution happened.
    // for future iterations of loop()
    ls_ts = millis();

  }

  // Just to test this process:
  // Alternate the motor activation 
  // every 2 seconds so that the 3Pi+
  // drives fowards, then backwards.
  elapsed_t = current_ts - motor_ts;
  if( elapsed_t > MOTOR_UPDATE ) {
    // Toggle motor direction
    // ...
    pwm_value *= -1;
    Serial.println(pwm_value);

    // Write motor direction and
    // pwm to motors.
    // ...

    motors.setMotorPower("left", pwm_value);
    motors.setMotorPower("right", pwm_value);

    // Record when this execution happened
    // for future iterations of loop()
    motor_ts = millis();

  }

  // Delay no longer being used.
  // delay(100)

}
