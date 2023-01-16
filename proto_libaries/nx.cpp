#include "nx.h"
#include "macros.h"
#include "i2cEeprom.h"

/* STEPS TO MAKE
V todo get the street from EEPROM
* make a new branch and make project compilable
* remake the .ino file to work with the new functions.
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

I2cEeprom   eeprom( 0x50 ) ;

const int   nPointsPerStreet = 13 ;
const int   sizeAddress  = 0x6FFE ;
const int   startAddress = 0x6FFF ; // should be the last free 4096 bytes memory in a 24LC256 EEPROM

const int   NA   = 0xFF ;
const int   last = 10000 ;
const int   maxCombinations = 128 ;

struct
{
    uint8_t     firstButton ;
    uint8_t     secondButton ;
    uint16_t    point[17] ;
    uint16_t    relay[5] ;
} Route;

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
        I2cEeprom.write( eeAddress++, *data ++ ) ;
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
        if( firstButton != NA ) { state = getSecondButton ; }
        break ;

    case getSecondButton:
        if(  firstButton == NA ) { state = getFirstButton ; }                   // if the first button is no longer pressed, go back
        if( secondButton != NA ) { state = getIndex ; }                         // if second button is also pressed, go find the street index
        break ;

    case getIndex:                                                              // 2 buttons are found, go find out which street matches with these buttons 
        uint16_t eeAddress = startAddress ;
        for( int i = 0 ; i < nStreets ; i ++ )
        {
            eeprom.get( eeAddress, Route ) ;                                    // fetch route from EEPROM
            eeAddress += sizeof( Route ) ;

            if((  firstButton == Route.firstButton && secondButton == Route.secondButton )
            || ( secondButton == Route.firstButton &&  firstButton == Route.secondButton && directionMatters ))
            {
                index = 0 ;
                firstButton  = NA ;     // reset buttons
                secondButton = NA ;
                state = setRoute ;
                return ;
            }
        }

        state = getFirstButton ;        
        break ;

    case setRoute:
        REPEAT_MS( 250 )
        {
            uint16_t point = Route.point[index++] ; // get point from array

            if( point == last )  // finished
            {
                if( relaisPresent )
                {
                    state = setRelays ;
                }
                else if( freeRouteOnTrain )
                {
                    state = waitDepature ;   
                    if( routeSet ) routeSet() ;
                }
                else
                {
                    state = getFirstButton ; 
                    if( routeSet )   routeSet() ; 
                    if( routeFreed ) routeFreed() ;
                }
                break ;
            }

            else   // setting turnouts
            {
                uint16_t    address     = point & 0x03FF ;
                uint8_t     pointState  = point >> 15 ;  
                if( setNxTurnout ) setNxTurnout( address, pointState ) ;
            }
        }
        END_REPEAT
        break ;

    case setRelays:
        index = 0 ;
        for( int i = 0 ; i < nRelais ; i ++ )
        {
            if( setNxRelay ) setNxRelay( i, 0 ) ; // kill all relais before setting new ones
        }

        uint16_t Route.relay[index++] ;
        if( relay == last )     // finished
        {
            if( routeSet )
            {
                routeSet() ;
            }
            if( freeRouteOnTrain )
            {
                state = waitDepature ;
            }
            else
            {
                state = getFirstButton ; 
                if( routeFreed ) routeFreed() ;
            }
            break ;
        }

        else        // seting relays
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
        if( speed == 0 ) 
        {
            if( routeFreed ) routeFreed() ;
            state = getFirstButton ;
        }
        break ;
    }
}

