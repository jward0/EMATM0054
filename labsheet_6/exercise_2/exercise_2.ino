#include "encoders.h"

void setup() {
  setupEncoderRight();
  setupEncoderLeft();
  Serial.begin(9600);
  delay(1000);

}

void loop() {

  Serial.println(count_e_right);
  Serial.println(count_e_left);
  Serial.println("-------------------------------------");
  delay(100);
}
