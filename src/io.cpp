#include <Arduino.h>
#include "io.h"
#include <Wire.h>

#define portA	0x12
#define portB	0x13
#define iodirRegA	0x00
#define pullUpRegA	0x0C

static void initMcp(unsigned char address, unsigned int iodir, unsigned int pullUp) {
	Wire.beginTransmission(address);
	Wire.write(iodirRegA);
	Wire.write(iodir >> 8);
	Wire.write(iodir);
	Wire.endTransmission();

	Wire.beginTransmission(address);
	Wire.write(pullUpRegA);
	Wire.write(pullUp >> 8);
	Wire.write(pullUp);
	Wire.endTransmission();
}

extern void mcpWrite(unsigned char pin, unsigned char state) {
	unsigned char address = 0x20, port, IO;
	address += (pin / 16);				// select address

	if(pin % 16 < 8)	port = portA;	// select port
	else				port = portB;
	
	pin %= 8 ;

	Wire.beginTransmission(address);
	Wire.write(port);
	Wire.endTransmission();
	Wire.requestFrom(address, 1);
	IO = Wire.read();					// fetch current IO status

	Wire.beginTransmission(address);
	Wire.write(port);
	if(state)	IO |=  (1 << pin);		// OR 'IO' with pin
	else		IO &= ~(1 << pin);		// or AND the inverse of pin
	Wire.write(IO);						// transmit the updated IO
	Wire.endTransmission();
}

extern unsigned char mcpRead(unsigned char pin) {
	unsigned char address = 0x20, port, IO;
	address += (pin / 16);				// select address

	if((pin % 16) < 8)	port = portA;	// select port
	else				port = portB;
	
	pin %= 8 ;

	Wire.beginTransmission(address);
	Wire.write(port);
	Wire.endTransmission();
	Wire.requestFrom(address, 1);
	IO = Wire.read();

	if(IO & (1 << pin))	return 1;
	else				return 0;
}

extern void initIO(void) {
	Wire.begin();
	pinMode(RS485DIR, OUTPUT);
	pinMode(servoPin1, OUTPUT);
	pinMode(servoPin2, OUTPUT);
	pinMode(servoPin3, OUTPUT);
	pinMode(servoPin4, OUTPUT);
	pinMode(servoPin5, OUTPUT);
	pinMode(servoPin6, OUTPUT);
	pinMode(servoPin7, OUTPUT);
	pinMode(servoPin8, OUTPUT);
	pinMode(statusLed, OUTPUT);
	pinMode(pinNumber, INPUT);
	pinMode(pwmPin1, OUTPUT);
	pinMode(pwmPin2, OUTPUT);
	pinMode(shortCircuit, INPUT);
	initMcp(0x20, 0b1111111111111111, 0b1111111111111111);
}