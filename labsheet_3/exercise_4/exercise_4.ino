#define LS_LEFT_PIN 18
#define LS_CENTRE_PIN 20
#define LS_RIGHT_PIN 21
#define EMIT_PIN 11

#define NB_LS_PINS 3

#define TIMEOUT_uS 10000

int ls_pins[NB_LS_PINS] = {LS_LEFT_PIN, LS_CENTRE_PIN, LS_RIGHT_PIN};


void setup() {
  
  Serial.begin(9600);
  
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, HIGH);

  pinMode(LS_LEFT_PIN, INPUT);
  pinMode(LS_CENTRE_PIN, INPUT);
  pinMode(LS_RIGHT_PIN, INPUT);

  delay(1000);
  Serial.println("Setup completed.");
  
}


void loop() {

  unsigned long exec_time_start;
  exec_time_start = micros();

  int i;

  for(i = 0; i < NB_LS_PINS; i++) {
    pinMode(ls_pins[i], OUTPUT);
    digitalWrite(ls_pins[i], HIGH);
  }

  delayMicroseconds(10);

  for(i = 0; i < NB_LS_PINS; i++) {
    pinMode(ls_pins[i], INPUT);
  }  

  unsigned long start_time;
  unsigned long end_times[NB_LS_PINS];
  bool pins_read[NB_LS_PINS] = {false, false, false};

  start_time = micros();
  
  while(true) {
    
    for(i = 0; i < NB_LS_PINS; i++) {
      
      if(digitalRead(ls_pins[i]) == LOW && !pins_read[i]) {
        end_times[i] = micros();
        pins_read[i] = true;
      }
    
      if(pins_read[0] == true && pins_read[1] == true && pins_read[2] == true) {
        break;
      }

      if(micros() - start_time > TIMEOUT_uS) {
        Serial.print("Error: Timeout value (");
        Serial.print(TIMEOUT_uS);
        Serial.println(" uS) exceeded.");
        break;
      }
    }
  }

  unsigned long elapsed_times[NB_LS_PINS];

  for(i = 0; i < NB_LS_PINS; i++) {
    elapsed_times[i] = end_times[i] - start_time;
  }

  Serial.println("Line sensors readings:");
  for(i = 0; i < NB_LS_PINS; i++) {
    Serial.println(elapsed_times[i]);
  }

  unsigned long exec_time_end;
  exec_time_end = micros();

  Serial.println(exec_time_end - exec_time_start);
  
  delay(100);
}
