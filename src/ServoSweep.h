#include <Arduino.h>
#include <Servo.h>

#define nTurnouts 8

class ServoSweep {

public:
    ServoSweep( uint8_t , uint8_t  ) ;        // constructor 1
    void sweep( );
    void setState( uint8_t _state );
    // void turnOn() ;
    // void turnOff() ;
    void begin( );
    void setMin( uint8_t ) ;
    void setMax( uint8_t ) ;
    uint8_t getMin( ) ;
    uint8_t getMax( ) ;
    uint8_t getState( ) ;

private:
    Servo servo ;
    unsigned long timeToRun ;
    int8_t pos ;
    uint8_t state ;
    uint8_t prevPos;
    uint8_t servoPin ;
    uint8_t servoSpeed ;
    uint8_t servoMin ;
    uint8_t servoMax  ;
    uint8_t middlePosition ;
    uint8_t relayPresent ;
    uint8_t relayPin ;
    uint8_t turnOff ;
    unsigned long currentTime = millis() ;
} ;

extern void initServos() ;
extern void sweepServos() ;
extern void adjustServo( int8_t ) ;
extern void setServo( uint8_t, uint8_t ) ;
