#include <Arduino.h>
#include <Wire.h>

const int   FIRST_BUTTON    = 1 ;
const int   SECOND_BUTTON   = 2 ;

const int   STRAIGHT        = 0 ;
const int   CURVED          = 1 ;
const int   OFF             = 2 ;

const uint8_t GPIO[] =
{
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    A0,
    A1,
    A2,
    A3,
} ;

extern const uint8_t nPoints ;
extern const uint8_t nButtons ;


// CALLBACK FUNCTIONS
/**
 * @brief is called to update an LED on me or slave
 * @param ledPin
 * @param state
 */
extern void setLed( uint8_t, uint8_t ) __attribute__((weak)) ;

/**
 * @brief is called by library to set a turnout
 * @param address of turnout
 * @param state of turnout
 */
extern void setNxTurnout( uint16_t, uint8_t ) __attribute__((weak)) ;


/**
 * @brief called by library to set a relay
 * @param address of relay
 * @param state of relay
 */
extern void setNxRelay(   uint16_t, uint8_t ) __attribute__((weak)) ;

/**
 * @brief called by library if the route is laid in
*/
extern void routeSet( ) __attribute__((weak)) ;


/**
 * @brief called by library if the route is freed up
*/
extern void routeFreed( ) __attribute__((weak)) ;


/**
 * @brief called by library if data from EEPROM is corrupted
*/
extern void invalidData( ) __attribute__((weak)) ;


class NxButton
{
private:
    uint8_t pin ;
    uint8_t state : 2;
    uint8_t i2c : 2;

public:
    NxButton( uint8_t _pin )
    {
        pin = _pin ;
        if( pin > 15 ) i2c = 1 ;
    }

    void setLed( uint8_t _state )
    {
        state = _state ;

        if( i2c ) // if not this board, relay info to slave
        {
            Wire.beginTransmission( pin/15 ) ; // slave address 
            //Wire.write( taskSetLed  ) ; // todo
            Wire.write( pin%15 ) ;
            Wire.write( state ) ;
            Wire.endTransmission() ;
        }
        else
        {
            if( state )
            {
                pinMode( GPIO[pin], INPUT_PULLUP ) ;          // if IO pin is held low, we cannot read it, so we release the pin
                digitalWrite( GPIO[pin], LOW ) ;      
            }
            else
            {
                pinMode( GPIO[pin], INPUT_PULLUP ) ;
            }
        }
    }

    uint8_t getPin()
    {
        return pin ;
    }
};



class Point
{
private:
    uint8_t     pin : 6 ; // 64 IO
    uint8_t     state : 2 ;  // 3 states
    uint16_t    dccAddress ;

public:
    Point( uint8_t _ledPin, uint16_t _dccAddress )
    {
        pin = _ledPin ;
        dccAddress = _dccAddress ;
    }

    void setLed( uint8_t _state )  // change me to call extern already present setLed function
    {
        if( pin >= 15 )
        {
            uint8_t slave = pin / 15 ; // slave
            pin           = pin % 15 ; // led number on slave
            Wire.beginTransmission( slave ) ;
            Wire.write( pin ) ;
            Wire.write( state ) ;
            Wire.endTransmission() ;
        }

        else switch( state )
        {
        case OFF:       pinMode( GPIO[pin], INPUT ) ; break ;

        case STRAIGHT:  pinMode( GPIO[pin], OUTPUT ) ;

                        digitalWrite( GPIO[pin], LOW ) ; break ;
        case CURVED:    pinMode( GPIO[pin], OUTPUT ) ;

                        digitalWrite( GPIO[pin], HIGH ) ; break ;
        }
    }

    uint8_t     getState()    { return state ; }
    uint8_t     getpin()      { return pin ; }
    uint16_t    getAddress()  { return dccAddress ; }
} ;



const int FREE_ON_TRAIN         = 0b00000001 ; // free route if train speed is set to 0
const int TURN_OFF_POINT_LED    = 0b00000010 ; // turn off all point LEDs before a route is set
const int HAS_RELAYS            = 0b00000100 ; // for analog layouts additional relays may be turned on
const int DIRECTION_MATTERS     = 0b00001000 ; // if direction of travel matters and have separate arrays
const int FREE_ON_BUTTON        = 0b00010000 ; // free a route if any button is pressed
const int TURN_ON_POINT_LED     = 0b00100000 ; // if points must be illuminated when route is available 
const int HAS_EEPROM            = 0b01000000 ; // if an I2C EEPROM is present, do not use Routes array

// N.B. if neither of the FREE options are used, the route is freed up directly after being set.

class NX
{
public:    
    Nx() ;
    void        run() ;
    void        begin( uint8_t ) ;
    void        setSpeed( int8_t ) ;
    void        setButton( uint8_t _btn, uint8_t state ) ;

private:

    uint32_t    prevTime ;
    uint8_t     flags ;
    int8_t      speed ;

    uint8_t     state ;
    uint8_t     firstButton ;
    uint8_t     secondButton ;
    uint8_t     index ;
    uint8_t     street ;
} ;

extern Point point[] ;
extern NxButton button[] ;