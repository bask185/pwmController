/* TODO
make branch for control panel usage

Add new file for the NX code.
Also do rosedale abbey
Multimaus must still be able to control servo motors in order to finetune.
default positions must be loaded to EEPROM if not yet done.
check pwm controller

*/

#include "src/macros.h"
#include "src/debounceClass.h"
#include "src/event.h"
#include "nx.h"


int8_t speedActual ;

#ifdef DEBUG
    #define debug Serial
#else
SoftwareSerial debug(9,10) ;
#include "src/XpressNetMaster.h"
XpressNetMasterClass    Xnet ;
#endif

NX nx ;

void notifyXNetLocoDrive128( uint16_t Address, uint8_t Speed )
{
    speedActual = Speed & 0x7F ;
    int8 direction   = Speed >> 7 ;
  
    speedActual = map(speedActual, 0, 127, 0, 100 ) ;

    if( direction > 0 ) speedActual = -speedActual ;
}


void routeSet() // called when all points are set
{
}

void routeFreed() // called when route is freed up
{
}

void setNxTurnout() // called when Nx modules wants to flip a switch.
{
}


void setup()
{
    nx.begin( 
      //  FREE_ON_TRAIN
        TURN_OFF_POINT_LED
      //| HAS_RELAYS
      //| DIRECTION_MATTERS
      | FREE_ON_BUTTON
      //| TURN_ON_POINT_LED 
    ) ;

#ifndef DEBUG
    Xnet.setup( loco128, 2 )
#else
    debug.begin( 9600 ) ;
    debug.println("PWM controller booted") ;
#endif
}


void loop()
{
    #ifndef DEBUG
    Xnet.update() ;
    #endif
    nx.run() ;
    // program.update() ; // not yet in use
}


#ifndef DEBUG
void notifyXNetTrnt(uint16_t Address, uint8_t data)
{
    if( bitRead( data, 3 ) == 1 )
    {
        // fill me in
    }
}
#endif