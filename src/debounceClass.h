#include <Arduino.h>

#ifndef button_h
#define	button_h

// //#define 
// #define ON 9 // random numbers, RISING and FALLING are already defined in Arduino.h
// #define OFF 10 // REPLACED FOR HIGH AND LOW

class Debounce {
public:
	Debounce(unsigned char _pin);
	unsigned char getState();
	void debounce();
	void debounce( bool );

private:
	unsigned char state;
	unsigned char pin; 
	bool oldSample = false;
	bool statePrev = false;
};
	
#endif