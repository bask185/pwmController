#include <Arduino.h>

#include <SoftwareSerial.h>
extern SoftwareSerial debug ;

class Weistra {
public:
    Weistra(uint8_t, uint8_t, uint8_t);
    Weistra(uint8_t, uint8_t, uint8_t, uint8_t);
    void begin();
    void update();
    void setSpeed(int8_t);
    void setState(uint8_t ) ;
    void currentMonitor( uint8_t ) ;
    uint8_t getState( ) ;

private:
    uint16_t  intervalTime ;
    uint16_t  newIntervalTime ;
    uint8_t   trackPin1 ;
    uint8_t   trackPin2 ;
    uint8_t   currentPin ;
    uint8_t   state ;
    uint8_t   counter ;
    uint8_t   counter1 ;
    int8_t    dutyCycle ;
    int8_t    newDutyCycle ;
    int8_t    speed ;
    uint8_t   Fmin ;
    uint8_t   Fmax ;
    uint32_t  prevTime = 0; 
    uint32_t  interval = 0; 
    uint32_t  shortTime ;
    bool      doublePinMode ;
    const int nSamples = 10 ;

    /*volatile */uint8_t *portx_p1 ;
    /*volatile */uint8_t *portx_p2 ;
};

