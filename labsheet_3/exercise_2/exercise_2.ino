#define LS_LEFT_IN_PIN 18
#define LS_CENTRE_IN_PIN 20
#define LS_RIGHT_IN_PIN 21
#define EMIT_PIN 11

void setup() {
  
  Serial.begin(9600);
  
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, HIGH);

  pinMode(LS_LEFT_IN_PIN, INPUT);
  pinMode(LS_CENTRE_IN_PIN, INPUT);
  pinMode(LS_RIGHT_IN_PIN, INPUT);

  delay(1000);
  Serial.println("Setup completed.");
  
}


void loop() {
  
  pinMode(LS_LEFT_IN_PIN, OUTPUT);
  digitalWrite(LS_LEFT_IN_PIN, HIGH);

  delayMicroseconds(10);

  pinMode(LS_LEFT_IN_PIN, INPUT);

  unsigned long start_time;
  unsigned long end_time;

  start_time = micros();

  while(digitalRead(LS_LEFT_IN_PIN) == HIGH) {
  }

  end_time = micros();

  unsigned long elapsed_time;
  elapsed_time = end_time - start_time;

  Serial.print("Left line sensor: ");
  Serial.println(elapsed_time);

  delay(100);
}
