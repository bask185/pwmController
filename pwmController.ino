#include "src/io.h"
#include "src/date.h"
#include "src/version.h"
#include "src/macros.h"
#include "src/debounceClass.h"
#include "src/event.h"
#include "src/servoSweep.h"
#include "src/weistra.h"
#include "src/XpressNetMaster.h"
#include "Wire.h"
#include <SoftwareSerial.h>

const int F1_F4 = 0 ;
const int F5_F8 = 0x80 ;
const int F11 = -10 ; // degrees at the time
const int F12 =  10 ;
const int SPEED_MAX = 50 ; 

enum events
{
    accessoryEvent ,
    speedEvent,
} ;


XpressNetMasterClass    Xnet ;
EventHandler eventHandler( 0x0000, 0x7FFF ) ;

Weistra pwmController( pwmPin1, pwmPin2, 30, 100 ) ;

SoftwareSerial debug(9,10) ;

const int nPrograms = 5 ;
uint8_t channel ;
EventHandler program[] =
{
    EventHandler( 0x0000, 0x1999, 0x50 ),  // 5 equal zones of memory for 24LC256 I2C EEPROM
    EventHandler( 0x1999, 0x1999, 0x50 ),
    EventHandler( 0x3332, 0x1999, 0x50 ),
    EventHandler( 0x4CCB, 0x1999, 0x50 ),
    EventHandler( 0x6664, 0x1999, 0x50 ),
    // 0x7000 <-> 0x7FFF reserved for other
} ;





/* things to do:

- get eventhandler to work
V test MCP inputs
- test PCF output (relais)
V test servo motors
V get XpressNet control functions to work
V test pwm controller working well in one try! 2 pin control.
- add shorcircuit code to weistra

BACKLOG
- add acceleration/decceleration to weistra 
*/

int8 speed = 0 ; ;
bool accelerating = true ;

const int mcpPins[] = {  // PCB correction for mcp23017 pins
  7, 6, 5, 4, 3, 2, 1, 0, 8, 9, 10, 11, 12, 13, 14, 15, 
} ;
uint8_t buttonState[16] ;

uint8 debounceIndex = 0 ;
Debounce input[] = 
{
    Debounce(255), Debounce(255), Debounce(255), Debounce(255),
    Debounce(255), Debounce(255), Debounce(255), Debounce(255),
    Debounce(255), Debounce(255), Debounce(255), Debounce(255),
    Debounce(255), Debounce(255), Debounce(255), Debounce(255),
} ;

void debounceInputs()
{
    REPEAT_MS( 5 )
    {
        byte pin = mcpPins[debounceIndex] ;
        byte state = mcpRead(pin) ;
        input[ debounceIndex].debounce( state ) ;
        if( ++ debounceIndex == 16 ) debounceIndex = 0 ;   
    }
    END_REPEAT

    for( int i = 0 ; i < 16 ; i ++ )
    {
        buttonState[i] = input[i].getState() ;
    }
}

uint8_t prevStates[6] = { 0, 0, 0, 0, 0, 0 } ;
int8_t setPoint = 0 ;
int8_t currentSpeed = 0 ;

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
    

    int8 speedActual = lookUpSpeed( Speed & 0b00011111 ) ;
    speedActual = map( speedActual, 0, 28, 0, SPEED_MAX ) ;           // map 28 speedsteps to 100 for weistra control
    if( Speed & 0x80 ) speedActual = -speedActual ;
    pwmController.setSpeed( speedActual ) ;
    program[channel].storeEvent( speedEvent, 123, speedActual ) ;
}
void notifyXNetLocoDrive128( uint16_t Address, uint8_t Speed )
{
    debug.println(Speed) ;

    int8 speedActual = Speed & 0x7F ;
    int8 direction   = Speed >> 7 ;
    
    if( speedActual == 0 )
    {
        pwmController.setSpeed( 0 ) ;
        return ;
    }
    
    speedActual = map(speedActual, 0, 127, 0, SPEED_MAX ) ;

    if( direction > 0 ) speedActual = -speedActual ;
    pwmController.setSpeed( speedActual ) ;
    program[channel].storeEvent( speedEvent, 123, speedActual ) ;
}



void setOutput( uint8_t Address, uint8_t functions )
{
    if( Address == 3) return ; // address 3 is unused
    // if( Address == 6 )
    // {
    //     if( functions & 0b0001 ) startPlaying() ;
    //     if( functions & 0b0010 ) stopPlaying() ;
    //     if( functions & 0b0100 ) startRecording() ;
    //     if( functions & 0b1000 ) stopRecording() ;
    //     return ;
    // }

    program[channel].storeEvent( accessoryEvent,  Address, functions ) ; 

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

            if(      ioNumber <=  8 ) { setServo( ioNumber - 1 , state ) ;  }             //  1 <->  8
            // else if( ioNumber <= 18 ) { digitalWrite( relay[ioNumber-11], state^1 ) ; }     // 11 <-> 18
            //                                                                                 // 21 <-> 28  address 3 not used.
            // else if( ioNumber <= 38 ) setRoute( ioNumber - 31 ) ;                           // 31 <-> 38
            // else if( ioNumber <= 48 ) ;                                                     // 41 <-> 48

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

void notifyXNetgiveLocoInfo(uint8_t UserOps, uint16_t Address)
{
    Xnet.SetLocoInfo(UserOps, 0x00, 0x00, 0x00); //UserOps,Speed,F0,F1
}

void notifyXNetgiveLocoFunc(uint8_t UserOps, uint16_t Address)
{
    Xnet.SetFktStatus(UserOps, 0x00, 0x00); //Fkt4, Fkt5
}

void processButtons()
{
    // buttons 0-2 are the program control buttons
    if( buttonState[0] == FALLING ) program[channel].startPlaying() ;
    if( buttonState[1] == FALLING ) program[channel].stopRecording() ;
    if( buttonState[2] == FALLING ) program[channel].startRecording() ;

    // buttons 3-7 are channel selection buttons
    for( int i = 3 ; i < 8 ; i ++ )
    {
        if( buttonState[i] == FALLING )
        {
            channel = i - 3 ;
        }
    }

    // buttons 8-15 are feedback buttons to control the programs with
    for( int i = 8 ; i < 16 ; i ++ )
    {
        if( buttonState[i] == FALLING )
        {
            program[channel].storeEvent( FEEDBACK, i, 1  ) ;        // for recording

            for( int j = 0 ; j < nPrograms ; j ++ )
            {
                program[j].sendFeedbackEvent( i ) ;                 // during playing. A feedback event is sent to all programs
            }
        }
    }
}

void notifyEvent( uint8 type, uint16 address, uint8 data )                            // CALL BACK FUNCTION FROM EVENT.CPP
{
    switch( type )
    {
        case speedEvent:        notifyXNetLocoDrive28(address, data) ;      break ; 
        case accessoryEvent:    setOutput( address, data ) ;                break ;
    }
}
void setup()
{
    Wire.begin() ;
    pwmController.begin() ;
    for( int i = 0 ; i < nPrograms ; i ++ )
    {   
        program[ i ].begin() ;
    }
    initIO() ;
    initServos() ;
    Xnet.setup( Loco28, RS485DIR ) ; // N.B. may need to change depending on what multimaus will do.
    //debug.begin(9600) ;
    debug.println("PWM controller booted") ;
}

void loop()
{
    Xnet.update() ;
    pwmController.update() ;
    debounceInputs() ;
    processButtons() ;
    sweepServos() ;
    for( int i = 0 ; i < nPrograms ; i ++ )
    {
        program[i].update() ;                 // run all 5 prorams
    }    
}
