// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:
  
	float feedback_signal;
	float error_signal;
	
	float k_p;
	float k_i;
	float k_d;
	
	float p_term;
	float i_term;
	float d_term;
  
    // Constructor, must exist.
    PID_c() {

    } 
	
	void initialise(float p, float i, float d) {
		k_p = p;
		k_i = i;
		k_d = d;
		
		p_term = 0;
		i_term = 0;
		d_term = 0;
	}
	
	float update(float demand, float measurement) {
		error_signal = demand - measurement;

		p_term = k_p * error_signal;
		i_term = 0;
		d_term = 0;
		
		return p_term + i_term + d_term;
	}
	
};



#endif