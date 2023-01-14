/* TODO
make branch for control panel usage

Add new file for the NX code.
Also do rosedale abbey
Multimaus must still be able to control servo motors in order to finetune.
default positions must be loaded to EEPROM if not yet done.
check pwm controller

*/

#include "src/io.h"
#include "src/date.h"
#include "src/version.h"
#include "src/macros.h"
#include "src/debounceClass.h"
#include "src/event.h"
#include "src/servoSweep.h"
#include "src/weistra.h"
#include "Wire.h"

#define mcpAddress 0x20
#define portA	0x12
#define portB	0x13
#define iodirRegA	0x00
#define pullUpRegA	0x0C



const int F1_F4 = 0 ;
const int F5_F8 = 0x80 ;
const int F11 = -3 ; // degrees at the time
const int F12 =  3 ;
const int SPEED_MAX = 50 ; 
uint8_t speedFeedback ;

const int nButtons = 16 ;

EventHandler program( 0x0000, 0x7FFF, 0x50 ) ;

enum events
{
    accessoryEvent ,
    speedEvent,
} ;

const int nPointsPerStreet = 8 ;

const int       C = 0x8000 ;          // CURVED
const int       S = 0x0000 ;          // STRAIGHT
const int       last = 10000 ;   // must be in the end of every line in table to flag that the last point is set.
extern uint8_t  relays ;


// PANEEL KOEN 3|S,                          last,
const uint16_t accessories[][nPointsPerStreet+3] =
{
//   knoppen   wissels + standen        laatste wissel    relais
    { 1, 6,   3|S,                          last,     1, 6,    last }, 
    { 1, 8,   3|S,   1|S,                   last,     1, 6,    last },
    { 2, 6,   3|C,   4|C,                   last,     2, 6,    last },
    { 2, 7,   4|S,                          last,     2, 7,    last },
    { 2, 8,   1|S, 3|C, 4|C,                last,     2, 6,    last },
    { 2, 9,   4|S, 2|S, 7|S,                last,     2, 7, 8, last },
    { 6, 8,   1|S,                          last,     6,       last },
    { 7, 9,   2|S, 7|S,                     last,     7, 9,    last },
    { 8, 3,   1|C, 2|S, 7|S, 5|C, 6|S,      last,     3,       last },
    { 8, 4,   1|C, 2|S, 7|S, 5|C, 6|C,      last,     4,       last },
    { 8, 5,   1|C, 2|S, 7|S, 5|S,           last,     5,       last },
    { 9, 3,   2|C, 7|C, 5|C, 6|S,           last,     8, 3,    last },
    { 9, 4,   2|C, 7|C, 5|C, 6|C,           last,     8, 4,    last },
    { 9, 5,   2|C, 7|C, 5|S,                last,     8, 5,    last },
} ;

const int nStreets = 14 ;  // 1   2   3   4   5   6   7

const uint8_t buttonLeds[] = {11, 14, 10, 10, 12, 13, 11} ;
const uint8_t pointLeds[] = {11, 14, 10, 10, 12, 13, 11} ;
// const int nStreets = sizeof( accessories[8] ) / sizeof( accessories[0][0] ) ; // calculate amount of streets, depending on the size of the table above

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


uint8 state = getFirstButton ;

uint8 firstButton, secondButton ;
uint8 index = 0 ;
uint8 street ;
int8_t speedActual ;

const int pcfAddress = 0x21;

Weistra pwmController( pwmPin1, pwmPin2, 50, 100 ) ;

#ifdef DEBUG
    #define debug Serial
#else
SoftwareSerial debug(9,10) ;
#include "src/XpressNetMaster.h"
XpressNetMasterClass    Xnet ;
#endif

uint8_t relays ;
uint8_t relaysPrev ;
int8_t  speed = 0 ;
bool accelerating = true ;

// const int mcpPins[] = {  // Software correction for physical mcp23017 pins
//   7, 6, 5, 4, 3, 2, 1, 0, 8, 9, 10, 11, 12, 13, 14, 15, 
// } ;

uint8_t buttonState[16] ;

uint8 debounceIndex = 0 ;
Debounce input(255) ;

void debounceInputs()
{
    uint16_t IO ;
    static uint16_t prevIO ;
    REPEAT_MS( 1000 )
    {
        Serial.println("getting IO");
        Wire.beginTransmission( mcpAddress ) ;
        Wire.write( portA ) ;
        Wire.endTransmission() ;
        Wire.requestFrom( mcpAddress, 2 ) ;
        IO = (Wire.read() << 8) | Wire.read() ;  

        Serial.println(IO,BIN); 

    }
    END_REPEAT

    for( int i = 0 ; i < 11 ; i ++ )
    {
        buttonState[i] = HIGH ;
    }

    if( IO != prevIO )
    {
        for( int i = 0 ; i < 11 ; i ++ )
        {
            if((    IO & (1<<i)) 
            != (prevIO & (1<<i)))
            {
                buttonState[i] = FALLING ;
                
            }
        }
        prevIO = IO ;
    }
}

uint8_t prevStates[6] = { 0, 0, 0, 0, 0, 0 } ;
int8_t  setPoint = 0 ;
int8_t  currentSpeed = 0 ;

uint8_t lookUpSpeed( uint8_t speed )
{
    switch( speed )
    {
    case 0b00000 : return  0 ;
    case 0b00010 : return  1 ;
    case 0b10010 : return  2 ;
    case 0b00011 : return  3 ;
    case 0b10011 : return  4 ;
    case 0b00100 : return  5 ;
    case 0b10100 : return  6 ;
    case 0b00101 : return  7 ;
    case 0b10101 : return  8 ;
    case 0b00110 : return  9 ;
    case 0b10110 : return 10 ;
    case 0b00111 : return 11 ;
    case 0b10111 : return 12 ;
    case 0b01000 : return 13 ;
    case 0b11000 : return 14 ;
    case 0b01001 : return 15 ;
    case 0b11001 : return 16 ;
    case 0b01010 : return 17 ;
    case 0b11010 : return 18 ;
    case 0b01011 : return 19 ;
    case 0b11011 : return 20 ;
    case 0b01100 : return 21 ;
    case 0b11100 : return 22 ;
    case 0b01101 : return 23 ;
    case 0b11101 : return 24 ;
    case 0b01110 : return 25 ;
    case 0b11110 : return 26 ;
    case 0b01111 : return 27 ;
    case 0b11111 : return 28 ;
    }
}

void notifyXNetLocoDrive28( uint16_t Address, uint8_t Speed )
{
    speedActual = lookUpSpeed( Speed & 0b00011111 ) ;
    speedFeedback = Speed ;
    speedActual = map( speedActual, 0, 28, 0, SPEED_MAX ) ;           // map 28 speedsteps to 100 for weistra control
    if( Speed & 0x80 ) speedActual = -speedActual ;
    pwmController.setSpeed( speedActual ) ;
    program.storeEvent( speedEvent, 123, speedActual ) ;
}

void notifyXNetLocoDrive128( uint16_t Address, uint8_t Speed )
{
    speedActual = Speed & 0x7F ;
    int8 direction   = Speed >> 7 ;

    speedFeedback = Speed ;
    
    if( speedActual == 0 )
    {
        program.storeEvent( speedEvent, 123, speedActual ) ;
        pwmController.setSpeed( 0 ) ;
        return ;
    }
    
    speedActual = map(speedActual, 0, 127, 0, SPEED_MAX ) ;

    if( direction > 0 ) speedActual = -speedActual ;
    pwmController.setSpeed( speedActual ) ;
    program.storeEvent( speedEvent, 123, speedActual ) ;
}



void setOutput( uint8_t Address, uint8_t functions )
{
    if( Address == 3) return ;

    program.storeEvent( accessoryEvent,  Address, functions ) ; 

    uint8_t number = 1 ;
    uint8_t indexShift = 0 ;
    if( functions & F5_F8 )
    {
        number = 5 ;
        indexShift = 3 ;
    }
    uint8_t ioNumber = number + ((Address - 1) * 10) ;

    // for  F1-F4
    // address == 1 -> number range =  1 -  4 -> points  1  -  4
    // address == 2 -> number range = 11 - 14 -> points  9  - 12
    // address == 3 -> number range = 21 - 24 -> relay   1  -  4

    // for F5-F8 
    // address == 1 -> number range =  1 -  4 -> points  5  -  8
    // address == 2 -> number range = 11 - 14 -> points 13  - 16
    // address == 3 -> number range = 21 - 24 -> relay   5  -  8

    for( int bitMask = 0b001 ; bitMask <= 0b1000 ; bitMask <<= 1 )
    {
        if( (functions & bitMask) != (prevStates[ Address+indexShift ] & bitMask) )        // check all 4 bits for F1 - F4, if atleast 1 bit has changed
        {
            prevStates[ Address+indexShift ] = functions & 0x0F ;

            uint8_t ioNumber = number + ((Address - 1) * 10) ;
            uint8_t state ;

            if( functions & bitMask ) state = 1 ;    // on
            else                      state = 0 ;    // off        

            if(      ioNumber <=  8 ) { setServo( ioNumber - 1 , state ) ;  }                   //  1 <->  8
            else if( ioNumber <= 18 ) { bitWrite( relays, ioNumber-11, state ) ; }              // 11 <-> 18
            //                                                                                  // 21 <-> 28  address 3 not used.
            // else if( ioNumber <= 38 ) setRoute( ioNumber - 31 ) ;                            // 31 <-> 38
            // else if( ioNumber <= 48 ) ;                                                      // 41 <-> 48

            return ;
        }

        number ++ ;
    }
}

void notifyXNetLocoFunc1( uint16_t Address, uint8_t Func1 ) { setOutput( Address, Func1 | F1_F4 ) ; } // called from Xnet library
void notifyXNetLocoFunc2( uint16_t Address, uint8_t Func2 ) { setOutput( Address, Func2 | F5_F8 ) ; }
void notifyXNetLocoFunc3( uint16_t Address, uint8_t Func )
{
    static uint8_t prevFunc = 0xFF ;

    if( (Func & 0b0100) != (prevFunc & 0b0100) ) { adjustServo( F11 ) ; }
    if( (Func & 0b1000) != (prevFunc & 0b1000) ) { adjustServo( F12 ) ; }

    prevFunc = Func ;
}

/* TO BE REPLACED BY SELF SHUTTING DOWN / REINSTATING POWER MANAGEMENT 
void notifyXNetPower(uint8_t State) 
{   
    if( State == csNormal )
    {
        digitalWrite( statusLed, HIGH ) ; 
        //throttle.setState( 1 ) ;
    }
    else
    {
        digitalWrite( statusLed,  LOW ) ; 
        //throttle.setState( 0 ) ;                                                    // turn off track power
        currentSpeed = 0 ;                                                          // reset speed as well so train does not jump away when power is enabled.
        //setPoint = 0 ;                                                              // ensures that the maus's knob needs to be turned before train moves again
    }
}
*/



void notifyEvent( uint8 type, uint16 address, uint8 data )                            // CALL BACK FUNCTION FROM EVENT.CPP
{
    switch( type )
    {
        case speedEvent:        pwmController.setSpeed( data) ; break ; 
        case accessoryEvent:    setOutput( address, data ) ;    break ;
    }
}

void updateRelay()
{
    if( relays != relaysPrev )
    {   relaysPrev = relays ;

        Wire.beginTransmission( pcfAddress ) ;
        Wire.write( relays ) ;       
        Wire.endTransmission() ;
    }
}



void setup()
{
    Wire.begin() ;

    pwmController.begin() ;
    pwmController.currentMonitor( shortCircuit ) ;
    program.begin() ;
    initServos() ;
    #ifndef DEBUG
    Xnet.setup( Loco28, RS485DIR ) ; // N.B. may need to change depending on what multimaus will do.
    #endif
   

    Wire.beginTransmission(mcpAddress);
    Wire.write(iodirRegA);
    Wire.write(0b11111111);
    Wire.write(0b10000000);
    Wire.endTransmission();

    debug.begin( 9600 ) ;
    debug.println("PWM controller booted") ;
}

void loop()
{

    #ifndef DEBUG
    Xnet.update() ;
    #endif
    pwmController.update() ;
    debounceInputs() ;
     runNx() ;
    sweepServos() ;
    updateRelay() ;
    program.update() ; // not yet in use
}



void runNx()    
{
    uint16  point ;
    uint16  relay ;
    uint16  address ;
    uint8   pointState ;

    switch( state )
    {
    case getFirstButton:
        for(int i = 0 ; i < nButtons ; i ++ )
        {
            if( buttonState[i] == FALLING )
            {
                firstButton = i+1 ;
                state = getSecondButton ;
                printNumberln("firstButton ", firstButton ) ;
                break ;
            }
        }
        break ;

    case getSecondButton:
        for(int i = 0 ; i < nButtons ; i ++ )
        {
            if( buttonState[i] == RISING  )
            { 
                state = getFirstButton ;
                // printNumberln("released ", firstButton ) ;
                break ;
            }
            if( buttonState[i] == FALLING )
            {
                secondButton = i+1 ;
                // printNumberln("secondButton ", secondButton ) ;
                state = getIndex ;
                break ;
            }
        }
        break ;

    case getIndex:                          // 2 buttons are found, go find out which street matches with these buttons 
        for( int i = 0 ; i < nStreets ; i ++ )
        {
            if(( accessories[i][0] == firstButton  && accessories[i][1] == secondButton )
            ||   accessories[i][0] == secondButton && accessories[i][1] == firstButton )
            {
                street = i ;
                // printNumberln("street ", firstButton ) ;
                
                index = 2 ;                 // reset index before setting new street
                state = setRoute ;
                break ;
            }
        }

        state = getFirstButton ;        
        break ;

    case setRoute:
        REPEAT_MS( 250 )
        {
            point = accessories[street][index++] ;

            if( point == last )
            {
                state = setRelays ;
                break ;
            }
            else
            {
                uint16_t address = point & 0x03FF ;
                pointState = point >> 15 ;  
                setServo( address, pointState ) ;
            }
        }
        END_REPEAT
        break ;

    case setRelays:
        relays = 0 ;

        relay = accessories[street][index++] ;
            if( relay == last )
            {
                state = waitDepature ;
                break ;
            }
            else
            {
                relays |= (1 << relay ) ;
                // printNumberln("relay set: ", relay ) ;
            }
        break ;

    case waitDepature:
        if( speedActual != 0 ) state = waitArrival ;
        break ;

    case waitArrival:
        if( speedActual == 0 ) state = getFirstButton ;
        break ;
    }
}

#ifndef DEBUG
void notifyXNetgiveLocoInfo(uint8_t UserOps, uint16_t Address)
{
    Xnet.SetLocoInfo(UserOps, 0x00, 0x00, 0x00); //UserOps,Speed,F0,F1
}

void notifyXNetgiveLocoMM(uint8_t UserOps, uint16_t Address)
{
    uint8_t F0 = 0, F1 = 0 ;

    if( Address == 2 )
    {
        F1 = relays >> 4 ;
        F0 = relays & 0x0F ;
    }
    if( Address == 1 )
    {
        for( int i = 0 ; i < 8 ; i ++ )
        {
            if( getServo(i) )
            {
                if( i < 4 ) F0 |= 1 <<  i ;
                else        F1 |= 1 << i-4 ;
            }
        }
    }

    Xnet.SetLocoInfoMM(UserOps, 0x04, speedFeedback, F0, F1, 0, 0); //Steps,Speed,F0,F1,F2,F3
}


void notifyXNetgiveLocoFunc(uint8_t UserOps, uint16_t Address)
{
    Xnet.SetFktStatus(UserOps, 0x00, 0x00); //Fkt4, Fkt5
}

void notifyXNetTrntInfo(uint8_t UserOps, uint8_t Address, uint8_t data)
{
    int adr = ((Address * 4) + ((data & 0x01) * 2));
    byte pos = data << 4;
    bitWrite(pos, 7, 1);   // command completed!
    bitWrite(pos, 1, 1);   // this should send information for 2 points, experiment with this
    bitWrite(pos, 3, 1);    
    Xnet.SetTrntStatus(UserOps, Address, pos);
}

void notifyXNetTrnt(uint16_t Address, uint8_t data)
{
    if( bitRead( data, 3 ) == 1 )
    {
        // fill me in
    }
}
#endif