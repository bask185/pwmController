#include "ServoSweep.h"
#include <EEPROM.h>


#define INIT_ADDR       0X00
#define ADDR_S1         0X01
#define ADDR_S2         0X03
#define ADDR_S3         0X05
#define ADDR_S4         0X07
#define ADDR_S5         0X09
#define ADDR_S6         0X0B
#define ADDR_S7         0X0D
#define ADDR_S8         0X0F


uint8_t lastSetServo ;

ServoSweep::ServoSweep( uint8_t _speed, uint8_t _pin )                   // constructor 1
{
    servoSpeed = _speed ;
    servoPin = _pin ;
    turnOff = 1 ;

}

void ServoSweep::setMin( uint8_t limit )
{
    servoMin = limit ;
    servo.attach( servoPin ) ;
}

void ServoSweep::setMax( uint8_t limit )
{
    servoMax = limit ;
    servo.attach( servoPin ) ;
}

uint8_t ServoSweep::getMin( )
{
    return servoMin ;
}

uint8_t ServoSweep::getMax( )
{
    return servoMax ;
}

void ServoSweep::begin()
{
    //pos = (servoMax - servoMin) / 2 ;
    //if( pos < 0 ) pos = -pos ;
    //servo.write( pos ) ;
    //servo.attach( servoPin ) ;
}

uint8_t ServoSweep::getState()
{
    return state ;
}

void ServoSweep::setState( uint8_t _state )
{
    state = _state ;
    servo.attach( servoPin ) ;
}

void ServoSweep::sweep ( )
{
    if( millis() > timeToRun ) {
        timeToRun = millis() + servoSpeed ;

        if( state == 1 && pos < servoMax ) pos ++ ;
        if( state == 0 && pos > servoMin ) pos -- ;

        if( prevPos != pos ) {
            prevPos  = pos ;

            servo.write( pos ) ;
            if( pos == servoMax || pos == servoMin ) servo.detach( ) ;
        }
    }
}

// wrapper functions, to be called from main

const int SPEED = 20 ;

ServoSweep servo[] =
{
    ServoSweep( SPEED,  3 ) ,
    ServoSweep( SPEED,  4 ) ,
    ServoSweep( SPEED,  5 ) ,
    ServoSweep( SPEED,  6 ) ,
    ServoSweep( SPEED,  7 ) ,
    ServoSweep( SPEED,  8 ) ,
    ServoSweep( SPEED,  9 ) ,
    ServoSweep( SPEED, 10 ) ,
} ;

void initServos()
{
    //EEPROM.write( INIT_ADDR,     0xFF ) ;

    if( EEPROM.read( INIT_ADDR ) == 0xFF )  // if true, load default values
    {  EEPROM.write( INIT_ADDR,     0xCC ) ;

        for( int i = 0 ; i < 8 ; i ++ ) 
        {
            EEPROM.write( ADDR_S1 + ( 2 * i )     ,  45 ) ;
            EEPROM.write( ADDR_S1 + ( 2 * i ) + 1 , 135 ) ;
        }
    }

    for( int i = 0 ; i < 8 ; i ++ ) 
    {
        uint8_t lowPos  = EEPROM.read( ADDR_S1 + ( 2 * i )     ) ;
        uint8_t highPos = EEPROM.read( ADDR_S1 + ( 2 * i ) + 1 ) ;

        servo[i].setMin( lowPos ) ;
        servo[i].setMax( highPos ) ;
        servo[i].begin() ;
    }
}

void setServo( uint8_t nServo, uint8_t state )
{
    lastSetServo = nServo ;                             // keep track which servo was last to set.
    servo[ nServo ].setState( state ) ;
}

void sweepServos()
{
    static uint8_t index = 0 ;
    servo[index].sweep() ;
    if( ++ index == 8 ) index = 0 ;
}

void adjustServo( int8_t F11_F12 )
{
    uint16_t eeAddress ;

    if( servo[ lastSetServo].getState() )                 // ADJUST EITHER LOW POSITION...
    {
        //eeAddress = ADDR_S1 + (lastSetServo * 2) ;
        uint8_t low = servo[ lastSetServo].getMin() + F11_F12 ;
        servo[ lastSetServo].setMin( low ) ;
        //EEPROM.write(eeAddress, low ) ;     
        //servo[ lastSetServo ].setState( 1 ) ;    
    }
    else
    {                                                   // ... OR HIGH POSITION
        //eeAddress = ADDR_S1 + (lastSetServo * 2) + 1;
        uint8_t high = servo[ lastSetServo].getMax() + F11_F12 ;
        servo[ lastSetServo].setMax( high ) ;
        //EEPROM.write(eeAddress, high ) ; 
       // servo[ lastSetServo ].setState( 0 ) ; 
    }
}