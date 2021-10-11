#define LS_LEFT_PIN 18
#define LS_CENTRE_PIN 20
#define LS_RIGHT_PIN 21
#define EMIT_PIN 11

#define NB_LS_PINS 3

#define TIMEOUT_uS 10000

int ls_pins[NB_LS_PINS] = {LS_LEFT_PIN, LS_CENTRE_PIN, LS_RIGHT_PIN};


class lineSensors_c {

	public:

	
	void initialise() {
		
		pinMode(EMIT_PIN, OUTPUT);
		digitalWrite(EMIT_PIN, HIGH);
		
		pinMode(LS_LEFT_PIN, INPUT);
		pinMode(LS_CENTRE_PIN, INPUT);
		pinMode(LS_RIGHT_PIN, INPUT);
		
		delay(100);
	}
	/*
	unsigned long readSensor(int i) {
		
		pinMode(ls_pins[i], OUTPUT);
		digitalWrite(ls_pins[i], HIGH);
		
		delayMicroseconds(10);
		
		pinMode(ls_pins[i], INPUT);
		
		unsigned long start_time;
		unsigned long end_time;
		
		start_time = micros()
		
		while(true) {
			if(digitalRead(ls_pins[i] == LOW) {
				
				end_time = micros()
				break;
				
			}
			else if(micros() - start_time > TIMEOUT_uS) {
				
				break;
				
			}
			
		unsigned long elapsed_time;
		elapsed_time = end_time - start_time;
		
		return elapsed_time;
	}
	*/
	
	void readAllSensors(unsigned long* arr) {

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
		bool loop_break = false;

		start_time = micros();
		  
		while(!loop_break) {
			
			for(i = 0; i < NB_LS_PINS; i++) {
			  
				if(digitalRead(ls_pins[i]) == LOW && !pins_read[i]) {
					end_times[i] = micros();
					pins_read[i] = true;
				}
			
				if(pins_read[0] == true && pins_read[1] == true && pins_read[2] == true) {
					loop_break = true;
					break;
				}

				if(micros() - start_time > TIMEOUT_uS) {
					Serial.print("Error: Timeout value (");
					Serial.print(TIMEOUT_uS);
					Serial.println(" uS) exceeded.");
					loop_break = true;
					break;
				}
			}
		}

		unsigned long elapsed_times[NB_LS_PINS];

		for(i = 0; i < NB_LS_PINS; i++) {
			arr[i] = end_times[i] - start_time;
		}		

		//return elapsed_times; 
	}
	
	float calculateLineError(unsigned long* ls_read_times) {
  
		unsigned long sum = 0;

		for(int i = 0; i < NB_LS_PINS; i++) {

			sum += ls_read_times[i];
		}

		float w_left = (ls_read_times[0] + 0.5*ls_read_times[1]) / sum;
		float w_right = (ls_read_times[2] + 0.5*ls_read_times[1]) / sum;
		float e_line = w_left - w_right;

		return e_line;
	}

};