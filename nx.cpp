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
    setExtraBlocks,
} ;

const int   NA  = 0xFF ;



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

void NX::setButton( uint8_t _btn, uint8_t state )
{
    if( state ) // a button is pressed
    {
        if( firstButton == NA ) firstButton  = _btn ; // first set first button
        else                    secondButton = _btn ; // if first button is known, set second
    }
    else        // a button is released
    {
        firstButton  = NA ;                            // if a button is released clear both
        secondButton = NA ;
    }
}

void NX::run()    
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
        //uint16_t eeAddress = startAddress ;
        for( int i = 0 ; i < nStreets ; i ++ )
        {
            if((  firstButton == Route.firstButton && secondButton == Route.secondButton )
            || ( secondButton == Route.firstButton &&  firstButton == Route.secondButton && (flags & DIRECTION_MATTERS) ))
            {
                index = 0 ;
                firstButton  = NA ;     // reset buttons
                secondButton = NA ;
                
                for( int i = 0 ; i < nPoints ; i ++ ) button[i].setLed( OFF ) ; // turn all Nx button LEDs off

                if( flags & TURN_OFF_POINT_LED )
                {
                    for( int i = 0 ; i < nPoints ; i ++ ) point[i].setLed( OFF ) ; // turn all turnout LEDs off
                }

                state = setRoute ;                                              // route found!
                return ;
            }
        }

        state = getFirstButton ;                                                // no route found
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

                point[pointIndex].setLed( state ) ;      // sets the leds on controlpanel
                uint16_t dccAddress = point[pointIndex].getAddress() ;

                if( setNxTurnout ) setNxTurnout( dccAddress, state ) ;     // call back function to flip a switch
            }
        }
        break ;

    case setExtraBlocks:
        break ;


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

