#include <Arduino.h>

const int   FIRST_BUTTON    = 1 ;
const int   SECOND_BUTTON   = 2 ;

const int   STRAIGHT        = 0 ;
const int   CURVED          = 1 ;
const int   OFF             = 2 ;

// INITIALIZATION  _t _relaisPresent, uint8_t _freeRouteOnTrain, uint8_t _directionMatters )
/**
 * @brief initializes the library
 * @param relaisPresent analog layouts can make use of relais for the begin and end tracks and tracks in between
 * @param freeRouteOnTrain frees up the route after the train has stopped moving, otherwise route is freed after being set
 * @param directionMatters if set, one must use separate routes for each directon of travel, otherwise buttons work ambiguous
 */
extern void NxBegin( uint8_t, uint8_t, uint8_t ) ;


/**
 * @brief stores route to I2C EEPROM
 * @param pointer point to passed array
 * @param length  length of data to store
 * */
extern void storeRoutes( uint8_t *, uint16_t ) ;



// CONTROL FUNCTIONS
/**
 * @brief Pass the first and second pressed buttons
 * @param id first or second button?
 * @param val number of button
 */
extern void setNxButton( uint8_t, uint8_t ) ;



/**
 * @brief Pass the speed to the NX handler to free up a route
 * @param speed speed of loco
 */
extern void setNxSpeed( int8_t ) ;



// ROUND ROBIN TASK
extern void runNx( ) ;



// CALLBACK FUNCTIONS
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
    uint8_t state ;

public:
    NxButton( uint8_t _pin )
    {
        pin = _pin ;
    }

    uint8_t read()
    {
        uint8_t retVal ;

        if( state )
        {
            pinMode( pin, INPUT_PULLUP ) ;          // if IO pin is held low, we cannot read it, so we release the pin
            retVal = digitalRead( pin ) ;           // than we read the button state
            pinMode( pin, OUTPUT ) ;                // and we hold down the pin to keep the LED on
            digitalWrite( pin, LOW ) ;      
        }
        else
        {
            pinMode( pin, INPUT_PULLUP ) ;
            retVal = digitalRead( pin ) ;
        }

        return retVal ;
    }

    void setLed( uint8_t _state )
    {
        state = _state ;

        read() ; // updates the IO registers for us, discard return value.
    }

    uint8_t getPin()
    {
        return pin ;
    }
};



class Point
{
private:
    uint8_t     ledPin : 6 ; // 64 IO
    uint8_t     state : 2 ;  // 3 states
    uint16_t    dccAddress ;

public:
    Point( uint8_t _ledPin, uint16_t _dccAddress )
    {
        ledPin = _ledPin ;
        dccAddress = _dccAddress ;
    }

    void setState( uint8_t _state )
    {
        state = _state ;

        switch( state )
        {
        case CURVED:    pinMode(      ledPin, OUTPUT ) ;
                        digitalWrite( ledPin, LOW ) ;       break ;
        case STRAIGHT:  pinMode(      ledPin, OUTPUT ) ;
                        digitalWrite( ledPin, HIGH ) ;      break ;
        case OFF:       pinMode(      ledPin, INPUT ) ;     break ;
        }
    }

    uint8_t     getState()    { return state ; }
    uint8_t     getpin()      { return ledPin ; }
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

private:
    void        debounceButtons() ;

    uint32_t    prevTime ;
    uint8_t     flags ;
    int8_t      speed ;

    uint8_t     state ;
    uint8_t     firstButton ;
    uint8_t     secondButton ;
    uint8_t     index ;
    uint8_t     street ;
} ;
