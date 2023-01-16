#include "nx.h"
#include "macros.h"
#include "i2cEeprom.h"

/* STEPS TO MAKE
- complete HAL functions
- complete callback functions
- add interface method to send an array and store it in EPPROM
- fix the leds for points
- implement macros for
  * with or without relays
  * with or without release on loco movement 
*/


enum states
{
    getFirstButton,
    getSecondButton,
    getIndex,
    setRoute,
    waitDepature,
    setRelays,
    waitArrival,
} ;


uint8_t     state = getFirstButton ;
uint8_t     firstButton ;
uint8_t     secondButton ;
uint8_t     index ;
uint8_t     street ;
uint8_t     relaisPresent ;
uint8_t     freeRouteOnTrain ;
uint8_t     directionMatters ;
int8_t      speed ;

const int   nPointsPerStreet = 13 ;
const int   sizeAddress = 0x6FFE ; // should be the last free 4097 bytes memory in a 24LC256 EEPROM

const int   NA   = 0xFF ;
const int   last = 10000 ;   // must be in the end of every line in table to flag that the last point is set.
const int   maxCombinations = 128 ;

// PANEEL KOEN 3|S,                          last,
uint16_t street[maxCombinations][nPointsPerStreet+3] ;

uint8_t nButtons ;
uint8_t nStreets ;

void NxBegin( uint8_t _relaisPresent, uint8_t _freeRouteOnTrain, uint8_t _directionMatters )
{
    relaisPresent    = _relaisPresent ;
    freeRouteOnTrain = _freeRouteOnTrain ;
    directionMatters = _directionMatters ;

    nStreets = I2cEeprom.read( sizeAddress) ;
    if( nStreets = 255 && invalidData ) invalidData() ; // if the size byte is larger than the the max amount of streets, warning should be displayed!
}

void storeRoutes( uint8_t *data, uint16_t size )
{
    uint16_t eeAddress = sizeAddress ;
    I2cEeprom.write( eeAddress++, size ) ;

    for( int i = 0 ; i <  size ; i ++ )
    {
        I2cEeprom.write( eeAddress++, *data ++ )
    }
}

void setNxButton( uint8_t id, uint8_t val )
{
    if( id ==  FIRST_BUTTON )  firstButton = val ; 
    if( id == SECOND_BUTTON ) secondButton = val ;
}

void setNxSpeed( int8_t _speed )
{
    speed = _speed ;
}

void runNx()    
{
    switch( state )
    {
    case getFirstButton:
        if( firstButton != 255 ) { state = getSecondButton ; }
        break ;

    case getSecondButton:
        if(  firstButton == NA ) { state = getFirstButton ; }                   // if the first button is no longer pressed, go back
        if( secondButton != NA ) { state = getIndex ; }                         // if second button is also pressed, go find the street index
        break ;

    case getIndex:                                                              // 2 buttons are found, go find out which street matches with these buttons 
        for( int i = 0 ; i < nStreets ; i ++ )
        {           
            if((  accessories[i][0] == firstButton  && accessories[i][1] == secondButton )
            || (( accessories[i][0] == secondButton && accessories[i][1] ==  firstButton ) && directionMatters ))
            {
                street = i ;            
                index = 2 ;
                state = setRoute ;
                return ;
            }
        }

        state = getFirstButton ;        
        break ;

    case setRoute:
        REPEAT_MS( 250 )
        {
            uint16_t point = accessories[street][index++] ;

            if( point == last )
            {
                if(      relaisPresent    ) state = setRelays ;
                else if( freeRouteOnTrain ) state = waitDepature ;
                else                        state = getFirstButton ;
                break ;
            }
            else
            {
                uint16_t    address     = point & 0x03FF ;
                uint8_t     pointState  = point >> 15 ;  
                if( setNxTurnout ) setNxTurnout( address, pointState ) ;
            }
        }
        END_REPEAT
        break ;

    case setRelays:
        for( int i = 0 ; i < nRelais ; i ++ )
        {
            if( setNxRelay ) setNxRelay( i, 0 ) ; // kill all relais before setting new ones
        }

        uint16_t relay = accessories[street][index++] ;
        if( relay == last )
        {
            if( freeRouteOnTrain )  state = waitDepature ;
            else                    state = getFirstButton ;
            break ;
        }
        else
        {
            uint16_t    address     = point & 0x03FF ;
            uint8_t     relayState  = point >> 15 ;  
            if( setNxRelay ) setNxRelay( address, relayState ) ;
        }
        break ;

    case waitDepature:
        if( speed != 0 ) state = waitArrival ;
        break ;

    case waitArrival:
        if( speed == 0 ) state = getFirstButton ;
        break ;
    }
}

void setLed( uint8_t pin, uint8_t state ) // CURVED, STRAIGHT, OFF
{
    uint8_t val ;
    uint8_t port  = portA ;
    uint8_t ioDir = ioDirA
    if( pin >= 8 )
    {
        pin -= 8 ;
        port  = portB ;
        ioDir = ioDirB ;
    }

    if( state > OFF ) // STRAIGHT OR CURVED
    {
        val = readRegister( port ) ;
        if( state = CURVED ) { bitWrite( val, pin, 1 ) ; } // curved
        else                 { bitWrite( val, pin, 0 ) ; } // straight
        writeRegister( port, val ) ;
        
        val = readRegister( ioDir ) ;  // set pin to output
        bitWrite( val, pin, 0 ) ;
        writeRegister( ioDirA, val ) ;
    }
    else    // OFF
    {
        val = readRegister( ioDir ) ;  // set pin to input (kills the leds)
        bitWrite( val, pin, 1 ) ;
        writeRegister( ioDir, val ) ;
    }
}