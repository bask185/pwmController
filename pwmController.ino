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
// #include "nx.h"


int8_t speedActual ;

#ifdef DEBUG
    #define debug Serial
#else
SoftwareSerial debug(9,10) ;
#include "src/XpressNetMaster.h"
XpressNetMasterClass    Xnet ;
#endif

//NX nx ;

uint8_t     slaveMode;
uint8_t     nInputs ; // retreived from EEPROM eventually
uint8_t     myAddress ;
uint8_t     nSlaves ; // read from I2C bus who answered
const uint8_t maxSlaves = 6 ;
uint16_t    inputs[maxSlaves] ;
uint16_t    myInput ;           // 2 bytes for when we are I2C slave

const uint8_t nPins = 15 ;
const uint8_t firstPin = 3 ;
uint8_t       GPIOstate[nPins] ;
Debounce GPIO[] =
{
    Debounce( firstPin +  0 ),
    Debounce( firstPin +  1 ),
    Debounce( firstPin +  2 ),
    Debounce( firstPin +  3 ),
    Debounce( firstPin +  4 ),
    Debounce( firstPin +  5 ),
    Debounce( firstPin +  6 ),
    Debounce( firstPin +  7 ),
    Debounce( firstPin +  8 ),
    Debounce( firstPin +  9 ),
    Debounce( firstPin + 10 ),
    Debounce( firstPin + 11 ),
    Debounce( firstPin + 12 ),
    Debounce( firstPin + 13 ),
    Debounce( firstPin + 14 ),
} ;


const int OFF = 2 ;
const int CURVED = 1 ;
const int STRAIGHT = 0 ;

void setLed( uint8_t led, uint8_t state )
{
    switch( state )
    {
    case OFF:
        pinMode( led+firstPin, INPUT ) ;
        break ;

    case STRAIGHT:
        pinMode( led+firstPin, OUTPUT ) ;
        digitalWrite( led+firstPin, LOW ) ;
        break ;

    case CURVED:
        pinMode( led+firstPin, OUTPUT ) ;
        digitalWrite( led+firstPin, HIGH ) ;
        break ;
    }
}

void routeSet() // called when all points are set
{
}

void routeFreed() // called when route is freed up
{
}

void setNxTurnout( uint8_t address, uint8_t state ) // called when Nx modules wants to flip a switch.
{
    
#ifndef DEBUG
    Xnet.setTrntPos( point[address-1].dccAdress, state, 1 ) ;
    delay( 20 ) ;
    Xnet.setTrntPos( point[address-1].dccAdress, state, 0 ) ;
#endif
}

void debounceInputs()
{
    REPEAT_MS( 20 )
    {
        for( int i = 0 ; i < nPins ; i ++ )
        {
            GPIO[i].debounce() ;                // debounce my own IO
        }

        if( myAddress == 0 )    // if I am master I must request IO status of slaves, they 
        {
            for( int i = 0 ; i < nSlaves ; i ++ )
            {
                Wire.requestFrom( i+1, 2 ) ;  // request 2 bytes of each slave
                inputs[i] = (Wire.read() ) << 8 | Wire.read() ; // N.B. must somehow be debounced togather with my own IO.
            }
        }
    }

    END_REPEAT

    for( int i = 0 ; i < nPins ; i ++ )
    {
        GPIOstate[i] = GPIO[i].getState() ; // fix me to a uin32_t and use the bits for this
    }
}

void processInputs()
{
    for( int i = 0 ; i < nPins ; i ++ )
    {
        if( GPIOstate[i] == FALLING ) 
        {
            if( myAddress ) myInput &= ~(1<<i) ;   // slave mode
            //else nx.setButton( i, true ) ;       // master mode
        }

        if( GPIOstate[i] == RISING )
        {
            if( myAddress ) myInput |= (1<<i) ;    // slave mode
           // else nx.setButton( i, false ) ;      // master mode
        }
    }
}

 // if request command is received, we must send back 2 bytes with all input status
void requestEvent()
{
    Wire.write( highByte(myInput) ) ;
    Wire.write(  lowByte(myInput) ) ;
}

// if a command is received we merely have to update LEDs
void receiveEvent( int nBytes )
{
    uint8_t   led = Wire.read() ;
    uint8_t state = Wire.read() ;

    setLed( led, state ) ;
}

void setup()
{
    const int addressPins = A7 ;
    int sample = analogRead( addressPins ) ;

    myAddress = 0 ;
    if(      sample >= 100 - 10 || sample <= 100 + 10 ) myAddress = 1 ;
    else if( sample >= 200 - 10 || sample <= 200 + 10 ) myAddress = 2 ;
    else if( sample >= 300 - 10 || sample <= 300 + 10 ) myAddress = 3 ;
    else if( sample >= 400 - 10 || sample <= 400 + 10 ) myAddress = 4 ;

    if( myAddress )
    {
        slaveMode = 1 ; 
        Wire.begin( myAddress ) ;
        Wire.onRequest( requestEvent );
        Wire.onReceive( receiveEvent );
    }
    else
    {
        Wire.begin() ;              // master mode
        /*
        nx.begin( 
        //  FREE_ON_TRAIN
            TURN_OFF_POINT_LED
        //| HAS_RELAYS
        //| DIRECTION_MATTERS
        | FREE_ON_BUTTON
        //| TURN_ON_POINT_LED 
        ) ;
        */
        uint8_t status ;
        for( int i = 0 ; i < maxSlaves ; i ++ ) // max slaves is 6 for the time being
        {
            
            Wire.beginTransmission( i+1 ) ;
            status = Wire.endTransmission() ;
            if( status == 0 ) nSlaves ++ ;      // count the amount of slaves who answer
            else break ;                        // if no respons, break
        }
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
    debounceInputs() ; 
    processInputs() ;

    if( !myAddress ) // 0 -> master
    {   
        #ifndef DEBUG
        Xnet.update() ;
        #endif
        //nx.run() ;
        // program.update() ; // not yet in use
    }
}


#ifndef DEBUG
void notifyXNetTrnt(uint16_t Address, uint8_t data)
{
    if( bitRead( data, 3 ) == 1 )
    {
        // fill me in
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
