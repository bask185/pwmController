#include "nX.h"
#include "src/macros.h"

const int nPointsPerStreet = 6 ;

const int       C = 0x8000 ;          // CURVED
const int       S = 0x0000 ;          // STRAIGHT
const int       lastPoint = 10000 ;   // must be in the end of every line in table to flag that the last point is set.
const int       lastRelay = 10000 ;   // must be in the end of every line in table to flag that the last array
extern uint8_t  relays ;

// PANEEL KOEN
const int accessories[][nPointsPerStreet+3] =
{
//   knoppen   wissels + standen    laatste wissel    relais
    { 1, 6,   3|S,                       lastPoint,     1, 6,    lastRelay }, 
    { 1, 8,   3|S, 1|S,                  lastPoint,     1, 8, 6, lastRelay }, 
    { 2, 6,   3|C, 4|C,                  lastPoint,     2, 6,    lastRelay },
    { 2, 7,   4|S,                       lastPoint,     2, 7,    lastRelay },
    { 2, 8,   1|S, 3|C, 4|C              lastPoint,     2, 8,    lastRelay },
    { 2, 9,   4|S, 2a|S, 2b|S,           lastPoint,     2, 9, 7  lastRelay },
    { 6, 8,   1|S,                       lastPoint,     6, 8,    lastRelay },
    { 7, 9,  2a|S, 2b|S,                 lastPoint,     7, 9,    lastRelay },
    { 8, 3,   1|C, 2a|S, 2b|S, 5|C, 6|S, lastPoint,     8, 3,    lastRelay },
    { 8, 4,   1|C, 2a|S, 2b|S, 5|C, 6|C, lastPoint,     8, 4,    lastRelay },
    { 8, 5,   1|C, 2a|S, 2b|S, 5|S,      lastPoint,     8, 5,    lastRelay },
    { 9, 3,  2a|C, 2b|C,  5|C, 6|S       lastPoint,     9, 3,    lastRelay },
    { 9, 4,  2a|C, 2b|C,  5|C, 6|C       lastPoint,     9, 4,    lastRelay },
    { 9, 5,  2a|C, 2b|C,  5|S,           lastPoint,     9, 5,    lastRelay },
} ;

const int nStreets = sizeof( accessories ) / sizeof( accessories[0][0] ) / nPointsPerStreet - 1 ; // calculate amount of streets, depending on the size of the table above

enum states 
{
    getFirstButton,
    getSecondButton,
    getIndex,
    setPoints,
    waitDepature,
    setRelays,
    waitArrival,
} ;

uint8 state = getFirstButton ;

uint8 firstButton, secondButton ;
uint8 index = 0 ;
uint8 street ;

void runNx()
{
    uint16 point ;
    uint16 relay ;
    uint16 address ;
    uint8  pointState ;

    for( int i = 0 ; i < nButtons ; i ++ )
    {
        uint8 btnState = button[i].getState() ;

        switch( state )
        {
        case getFirstButton:
            if( btnState == FALLING )
            {
                firstButton = i ;
                state = getSecondButton ;
            }
            break ;

        case getSecondButton:
            if( btnState == RISING  )
            { 
                state = getFirstButton ;
            }
            if( btnState == FALLING )
            {
                secondButton = i ;
                state = getIndex ;
            }
            break ;

        case getIndex:                          // 2 buttons are found, go find out which street matches with these buttons 
            for( int i = 0 ; i < nStreets ; i ++ )
            {
                if(( accessories[i][0] == firstButton  && accessories[i][1] == secondButton )
                ||   accessories[i][0] == secondButton && accessories[i][1] == firstButton )
                {
                    street = i ;
                    index = 2 ;                 // reset index before setting new street
                    state = setPoints ;
                    goto indexFound ;
                }
            }

            state = getFirstButton ;        
        indexFound:
            break ;

        case setPoints:
            REPEAT_MS( 250 )
            {
                point = accessories[street][index++] ;
                if( point == lastPoint )
                {
                    state = setRelays ;
                    break ;
                }
                else
                {
                    address = point & 0x03FF ;
                    pointState = point >> 15 ;  
                    if( setPoints ) setPoints( address, address ) ;
                }
            }
            END_REPEAT
            break ;

        case setRelays:
            relays = 0 ;

            relay = accessories[street][index++] ;
                if( relay == lastRelay )
                {
                    state = waitDepature ;
                    break ;
                }
                else
                {
                    relays |= (1 << relay ) ;
                }
            break ;

        case waitDepature:
            if(1) state = waitArrival ; // to be filled in
            break ;

        case waitArrival:
            if(1) state = getFirstButton ;
            break ;
        }
    }
} 