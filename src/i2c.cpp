#include "io.h"
#include <Wire.h>



i2c::i2c() {} // constructor

i2c::void begin( uint8_t _myAddress )
{
	myAddress = _myAddress ;
	if( myAddress )
	{
		slaveMode = 1 ; // set to slave mode

		Wire.onReceive(receiveEvent) ;
		Wire.onRequest(requestEvent) ;
	}
}


// master only
i2c::void i2cWrite( uint8_t pin, uint8_t state )
{
	if( slaveMode ) return ;

	uint8_t address = pin / 15 + 1 ; 	// if master mode, select slave address

	Wire.beginTransmission( address ) ;
	Wire.write( pin ) ;
	Wire.write( state ) ;
	Wire.endTransmission() ;
}

// master only
i2c::uint8_t i2cRead( uint8_t pin )
{
	if( slaveMode ) return ; // slaves may not read other's IO

	uint8_t address = pin / 15 + 1 ;
	
	pin %= 15 ;

	Wire.beginTransmission( address ) ;
	Wire.write( pin ) ;					// inform the slave which pin we want a reading from
	Wire.endTransmission() ;

	Wire.requestFrom( address,  1 ) ; 	// than we request the pin status

	return Wire.read() ;				// and return this status
}

// slave mode only
i2c::void receiveEvent( int nBytes )
{
	pin2send = Wire.read()  ;
	pinState = digitalRead( GPIO[pin2send] ) ;
}

// slave mode only
i2c::void requestEvent()
{
	Wire.write( pinState ) ;
}