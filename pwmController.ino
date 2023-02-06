/* TODO
make branch for control panel usage

Add new file for the NX code.
Also do rosedale abbey
Multimaus must still be able to control servo motors in order to finetune.
default positions must be loaded to EEPROM if not yet done.
check pwm controller

*/

#include "src/macros.h"
#include "src/event.h"
#include "nx.h"


/* TODO2
expand I2C protocol with initializations. Inputs and outputs must be configured

figure out how the NX and point classes are to work with I2C slaves



FUBAR, make special slave software, this is chaotic
*/

int8_t speedActual ;

#ifdef DEBUG
    #define debug Serial
#else
SoftwareSerial debug(9,10) ;
#include "src/XpressNetMaster.h"
XpressNetMasterClass    Xnet ;
#endif


NX nx ;

uint8_t         nInputs ; // retreived from EEPROM eventually
uint8_t         nSlaves ; // read from I2C bus who answered
const uint8_t   maxSlaves = 6 ;
uint16_t        input[maxSlaves+1] ;
uint16_t        inputPrev[maxSlaves+1] ;

const uint8_t   nPins = 15 ;
const uint8_t   firstPin = 3 ;
uint8_t         GPIOstate[nPins] ;

// NOTE this class is deemed needed in order to handle DCC addresses as well as IO
// The array INDEX determens which LED ID it is and not the IO number
// I2C messages are to be sent to slaves in order to control LEDs
// if I am a slave this point array should not do anything, it may exist
// every element consumes 3 bytes, dummy elements may be needed
Point point[] =
{
    Point( 2,  20 ), //  1 // led Pin, dcc address 
    Point( 3,  21 ), //  2 // if pin exceeds 15, the led is on I2C slave
    Point( 4,  22 ), //  3
    Point( 5,  23 ), //  4
    Point( 6,  24 ), //  5
    Point( 7,  25 ), //  6
    Point( 8,  26 ), //  7
    Point( 9,  27 ), //  8
    Point( 10, 28 ), //  9
    Point( 11, 29 ), //  10
} ;
const uint8_t nPoints = sizeof( point ) / sizeof( point[0] ) ;

NxButton button[] =
{
    NxButton( 12 ), // led/switch pin
    NxButton( 13 ),
    NxButton( 14 ),
    NxButton( 15 ),
    NxButton( 16 ), // use this number to find out we are on I2C slave
    NxButton( 17 ),
    NxButton( 18 ),
    NxButton( 19 ),
    NxButton( 20 ),
} ;
const uint8_t nButtons = sizeof( button ) / sizeof( button[0] ) ;


// NX CALLBACK
void routeSet() // called when all points are set
{

}

// NX CALLBACK
void routeFreed() // called when route is freed up
{

}

// NX CALLBACK
void setNxTurnout( uint8_t address, uint8_t state ) // called when Nx modules wants to flip a switch.
{
#ifndef DEBUG
    Xnet.setTrntPos( point[address-1].dccAdress, state, 1 ) ;
    delay( 20 ) ;
    Xnet.setTrntPos( point[address-1].dccAdress, state, 0 ) ;
#endif
}

// ROUNDROBIN TASK
void readInputs()
{
    REPEAT_MS( 20 )
    {
        input[0] = 0 ; // clear befor setting bits
        for( int i = 0 ; i < nPins ; i ++ )
        {
            uint8_t state = digitalRead( firstPin + i ) ;   // these are my own IO
            input[0] |= state << i ;
        }

        for( int i = 0 ; i < nSlaves ; i ++ )
        {
            Wire.requestFrom( i+1, 2 ) ;                    // request inputs of each slave
            input[i+1] = (Wire.read() ) << 8 | Wire.read() ;
        }
    }
    END_REPEAT
}

// ROUNDROBIN TASK
void processInputs() // check me
{
    for( int i = 0 ; i < nSlaves+1 ; i ++ ) // for myself + any slave
    for( int j = 0 ; j < 15        ; j ++ ) // for all 15 IO
    {
        if( input[i] != inputPrev[i] )  // if any pin has changed..
        {   inputPrev[i] = input[i] ;

            if( input[i] > inputPrev[i] ) nx.setButton( j, 1 ) ; // send number to Nx.cpp for route searching
            else                          nx.setButton( j, 0 ) ;
        }
    }
}


void setup()
{
    Wire.begin() ;              // master mode
    
    nx.begin( 
    //  FREE_ON_TRAIN
          TURN_OFF_POINT_LED
    //  | HAS_RELAYS
    //  | DIRECTION_MATTERS
        | FREE_ON_BUTTON
    //  | TURN_ON_POINT_LED 
    ) ;
    
    uint8_t status ;
    for( int i = 0 ; i < maxSlaves ; i ++ ) // max slaves is 6 for the time being
    {
        Wire.beginTransmission( i+1 ) ;
        status = Wire.endTransmission() ;
        if( status == 0 ) nSlaves ++ ;      // count the amount of slaves who answer
        else break ;                        // if no respons, break DO SOMETHING WITH ERROR
    }

#ifndef DEBUG
    Xnet.setup( loco128, 2 )
#else
    debug.begin( 9600 ) ;
    debug.println("PanelX controller booted") ;
#endif
}


void loop()
{
    readInputs() ;          // reads in all buttons, locals and of slaves
    processInputs() ;       // sends all button information to the Nx module

#ifndef DEBUG
    Xnet.update() ;         // handles Xnet communication
#endif
    nx.run() ;              // handles all NX logic as well as controlling the LEDs, local and on slaves

    // program.update() ; //handles playing of recorded programs.  NOT YET IN USE   
}


#ifndef DEBUG
void notifyXNetTrnt(uint16_t Address, uint8_t data) // cross reference to used point objects
{                                                   // and set states accordingly
    if( bitRead( data, 3 ) == 1 )
    {
        data &= 0b11 ;
        if( data > 2 ) data -= 2 ;

        for( int i = 0 ; i < nPoints ; i ++ )
        {
            if( point[i].getAddress() == Address ) // check if address is for one of me points
            {
                point[i].setLed( data ) ;
            }
        }
    }
}

void notifyXNetLocoDrive128( uint16_t Address, uint8_t Speed )
{
    speedActual = Speed & 0x7F ;
    int8 direction   = Speed >> 7 ;
  
    speedActual = map(speedActual, 0, 127, 0, 100 ) ;

    if( direction > 0 ) speedActual = -speedActual ;
}

#endif
