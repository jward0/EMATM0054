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

}

// Repeats.
void loop() {

  // A slow (low) level of activation to test
  // the motor operation.
  digitalWrite( L_DIR_PIN, FWD);
  digitalWrite( R_DIR_PIN, FWD);
  analogWrite( L_PWM_PIN, 11 );
  analogWrite( R_PWM_PIN, 11 );
  delay(1000);
  
  analogWrite( L_PWM_PIN, 0 );
  analogWrite( R_PWM_PIN, 0 );
  delay(1000);
  
  digitalWrite( L_DIR_PIN, REV);
  digitalWrite( R_DIR_PIN, REV);
  analogWrite( L_PWM_PIN, 11 );
  analogWrite( R_PWM_PIN, 11 );
  delay(1000);
  
  analogWrite( L_PWM_PIN, 0 );
  analogWrite( R_PWM_PIN, 0 );
  delay(1000);

  // An empty loop can block further uploads.
  // A small delay to prevent this for now.
  delay(5);
}
