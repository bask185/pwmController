#include "weistra.h"


Weistra::Weistra(uint8_t _pin, uint8_t _Fmin, uint8_t _Fmax ) 
{ // constructor
    doublePinMode = false ;
    trackPin1 = _pin;
    Fmin = _Fmin ;
    Fmax = _Fmax ;
}

Weistra::Weistra(uint8_t _pin1, uint8_t _pin2, uint8_t _Fmin, uint8_t _Fmax ) 
{ // constructor
    doublePinMode = true ;
    trackPin1 = _pin1;
    trackPin2 = _pin2;
    Fmin = _Fmin ;
    Fmax = _Fmax ;
} 

void Weistra::begin()
{
    pinMode(trackPin1, OUTPUT);
    byte port   = digitalPinToPort( trackPin1 );
    trackPin1   = digitalPinToBitMask( trackPin1 );
    portx_p1    = portOutputRegister( port );

    if( doublePinMode )
    {
        pinMode(trackPin2, OUTPUT);
        port        = digitalPinToPort( trackPin2 );
        trackPin2   = digitalPinToBitMask( trackPin2 );
        portx_p2    = portOutputRegister( port );
    }
}

void Weistra::update() 
{
    if( portx_p1 != 0 )
    {
        if( micros() - prevTime >= intervalTime )
        {           prevTime = micros();

            if( counter == 0 && newDutyCycle > 0 )      // if counter reaches 100, reset it to 0 and enable the track power pin
            {
                if(      speed > 0 ) *portx_p1 |=  trackPin1 ;
                else if( speed < 0
                &&  doublePinMode  ) *portx_p2 |=  trackPin2 ;

                dutyCycle = newDutyCycle ;              // a new dutycucle can only be accepted on the beginning of a cycle, this prevents weird jumps of the trains
                intervalTime = newIntervalTime ;        // new speed is accepted at the beginning of a cycle
            }
            if( counter == dutyCycle /*&& dutyCycle < 100*/ ) // commented code seems buggy??
            {
                *portx_p1 &= ~trackPin1;
                *portx_p2 &= ~trackPin2;
            }
            if( ++counter > 100) counter = 0 ;
        }
    }
}


void Weistra::setSpeed( int8_t _speed )
{
    
    speed = _speed ;
    int8_t speedTemp ;
    if( speed < 0 ) speedTemp = -speed ;
    else            speedTemp =  speed ;

    uint8_t frequency;
    newDutyCycle = constrain( speedTemp, 0, 100 ) ; // speed limit = 0 - 100

    if( newDutyCycle <= 10 ) { frequency = Fmin; }
    else                     { frequency = map( newDutyCycle, 10, 100, Fmin, Fmax ) ; }

    newIntervalTime = 10000 / frequency; // > between 100us and 500us
}

void Weistra::setState( uint8_t _state )
{ 
    state = _state ;
    
    if( !state )
    {
        *portx_p1 &= ~trackPin1 ;
        *portx_p2 &= ~trackPin2 ;
    }
    else
    {
        counter = 0 ;
    }
}

uint8_t Weistra::getState( ) 
{
    return state ;
}

