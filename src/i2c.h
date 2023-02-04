#include <Arduino.h>

class i2c
{
public:
	i2c() ;
	void 	begin( uint8_t myAddress ) ;
	void 	i2cWrite( uint8_t, uint8_t ) ;
	uint8_t i2cRead( uint8_t ) ;

private:
	uint8_t myAddress ;
	uint8_t mode ;
	uint8_t pin2send ;
	uint8_t pinState ;
	const uint8_t GPIO[] =
	{
		3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3
	} ;

	void 	requestEvent() ;
	void 	receiveEvent( int nBytes ) ;
};