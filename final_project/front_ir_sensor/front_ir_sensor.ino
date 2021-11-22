#define NB_IR_PINS 2
#include "irsensor.h"

irSensors_c ir_sensors;


void setup() {
  
  Serial.begin(9600);
  
  ir_sensors.initialise();

  delay(1000);
  Serial.println("Setup completed.");
  
}


void loop() {

  unsigned long exec_time_start;
  exec_time_start = micros();

  unsigned long elapsed_times[NB_IR_PINS];
  
  ir_sensors.readAllSensors(elapsed_times);
  
  Serial.println("IR sensors readings:");
  for(int i = 0; i < NB_IR_PINS; i++) {
    Serial.println(elapsed_times[i]);
  }

  unsigned long exec_time_end;
  exec_time_end = micros();

  Serial.println("-------------");
  Serial.println(exec_time_end - exec_time_start);
  Serial.println("-------------");
  
  delay(100);
}
