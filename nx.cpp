#include "nx.h"
#include "routes.h"
#include "src/macros.h"


enum states
{
    getFirstButton,
    getSecondButton,
    getIndex,
    setRoute,
    waitDepature,
    setRelays,
    waitArrival,
    waitButtonPress,
} ;

const int   NA  = 0xFF ;

Point point[] =
{
    Point( 2,  20 ), // led Pin, dcc address 
    Point( 3,  21 ),
    Point( 4,  22 ),
    Point( 5,  23 ),
    Point( 6,  24 ),
    Point( 7,  25 ),
    Point( 8,  26 ),
    Point( 9,  27 ),
    Point( 10, 28 ),
    Point( 11, 29 ),
} ;
const uint8_t nPoints = sizeof( point ) / sizeof( point[0] ) ;

NxButton button[] =
{
    NxButton( 12 ), // led/switch pin
    NxButton( 13 ),
    NxButton( 14 ),
    NxButton( 15 ),
    NxButton( 16 ),
    NxButton( 17 ),
    NxButton( 18 ),
    NxButton( 19 ),
    NxButton( 20 ),
} ;
const uint8_t nButtons = sizeof( button ) / sizeof( button[0] ) ;

uint8_t nStreets ;

void storeRoutes( uint8_t *data, uint16_t size )
{
    // uint16_t eeAddress = sizeAddress ;
    // I2cEeprom.write( eeAddress++, size ) ;

    // for( int i = 0 ; i <  size ; i ++ )
    // {
    //     I2cEeprom.write( eeAddress++, *data ++ ) ;
    // }
}



void NX::begin( uint8_t _flags )
{
    flags = _flags ;
}

void NX::setSpeed( int8_t _speed )
{
    speed = _speed ;
}

void NX::debounceButtons()
{
}

void NX::run()    
{
    debounceButtons() ;

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
        //uint16_t eeAddress = startAddress ;
        for( int i = 0 ; i < nStreets ; i ++ )
        {
            if((  firstButton == Route.firstButton && secondButton == Route.secondButton )
            || ( secondButton == Route.firstButton &&  firstButton == Route.secondButton && (flags & DIRECTION_MATTERS) ))
            {
                index = 0 ;
                firstButton  = NA ;     // reset buttons
                secondButton = NA ;
                state = setRoute ;

                if( flags & TURN_OFF_POINT_LED )
                {
                    for( int i = 0 ; i < nPoints ; i ++ ) point[i].setState( OFF ) ;
                }
                return ;
            }
        }

        state = getFirstButton ;        
        break ;

    case setRoute:
        if( millis() - prevTime >= 250 )
        {       prevTime = millis() ;
            
            uint16_t pointIndex = Route.point[index++] ; // get point number from array

            if( pointIndex == last )  // finished
            {
                if( flags & HAS_RELAYS )
                {
                    state = setRelays ;
                }

                else if( flags & FREE_ON_TRAIN )
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
                uint8_t state = pointIndex >> 15 ;  // MSB is the state
                pointIndex &= 0x3FF ;               // 10 LSB carries point number
                pointIndex -- ;                     // array is zero indexed

                point[pointIndex].setState( state ) ;      // sets the leds on controlpanel
                uint16_t dccAddress = point[pointIndex].getAddress() ;

                if( setNxTurnout ) setNxTurnout( dccAddress, state ) ;     // call back function to flip a switch
            }
        }
        break ;

    case setExtraBlocks:


    case setRelays:
    /*
        index = 0 ;
        for( int i = 0 ; i < nRelais ; i ++ )
        {
            if( setNxRelay ) setNxRelay( i, 0 ) ; // kill all relais before setting new ones
        }

        uint16_t relayIndex = Route.point[index++] ; // get point number from array

        uint16_t Route.relay[index++] ;
        if( relay == last )     // finished
        {            
            if(      flags & FREE_ON_TRAIN  ) { state = waitDepature ; }
            else if( flags & FREE_ON_BUTTON ) { state = waitButtonPress ; }
            else                              { state = getFirstButton ;  
                                                if( routeFreed ) routeFreed() ; }
                                                
            if( routeSet ) routeSet() ;
            break ;
        }

        else        // seting relays
        {
            uint16_t    address     = point & 0x03FF ;
            uint8_t     relayState  = point >> 15 ;  
            if( setNxRelay ) setNxRelay( address, relayState ) ;
        }
        */
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

    case waitButtonPress:
        if( 1 ) // use anybutton to free up route.
        {
            if( routeFreed ) routeFreed() ;
            state = getFirstButton ;
        }
        break ;
    }
}

