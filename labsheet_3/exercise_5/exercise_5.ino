#define NB_LS_PINS 3
#include "linesensor.h"

lineSensors_c line_sensors;


void setup() {
  
  Serial.begin(9600);
  
  line_sensors.initialise();

  delay(1000);
  Serial.println("Setup completed.");
  
}


void loop() {

  unsigned long exec_time_start;
  exec_time_start = micros();

  unsigned long elapsed_times[NB_LS_PINS];
  
  line_sensors.readAllSensors(elapsed_times);
  
  Serial.println("Line sensors readings:");
  for(int i = 0; i < NB_LS_PINS; i++) {
    Serial.println(elapsed_times[i]);
  }

  unsigned long exec_time_end;
  exec_time_end = micros();

  Serial.println(exec_time_end - exec_time_start);
  
  delay(100);
}
