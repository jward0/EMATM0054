#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH

// Runs once.
void setup() {

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
  
  digitalWrite( L_DIR_PIN, FWD);
  digitalWrite( R_DIR_PIN, FWD);
  analogWrite( L_PWM_PIN, 40 );
  analogWrite( R_PWM_PIN, 40 );
  delay(3500);
  analogWrite( L_PWM_PIN, 0 );
  analogWrite( R_PWM_PIN, 0 );

}

// Repeats.
void loop() {
  delay(5);

}
