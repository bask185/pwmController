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


const int F1_F4 = 0 ;
const int F5_F8 = 0x80 ;
const int F11 = -3 ; // degrees at the time
const int F12 =  3 ;
const int SPEED_MAX = 50 ; 

enum events
{
    accessoryEvent ,
    speedEvent,
} ;

const int nPointsPerStreet = 8 ;

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
    { 2, 8,   1|S, 3|C, 4|C,             lastPoint,     2, 8,    lastRelay },
    { 2, 9,   4|S, 2a|S, 2b|S,           lastPoint,     2, 9, 7, lastRelay },
    { 6, 8,   1|S,                       lastPoint,     6, 8,    lastRelay },
    { 7, 9,  2a|S, 2b|S,                 lastPoint,     7, 9,    lastRelay },
    { 8, 3,   1|C, 2a|S, 2b|S, 5|C, 6|S, lastPoint,     8, 3,    lastRelay },
    { 8, 4,   1|C, 2a|S, 2b|S, 5|C, 6|C, lastPoint,     8, 4,    lastRelay },
    { 8, 5,   1|C, 2a|S, 2b|S, 5|S,      lastPoint,     8, 5,    lastRelay },
    { 9, 3,  2a|C, 2b|C,  5|C, 6|S,      lastPoint,     9, 3,    lastRelay },
    { 9, 4,  2a|C, 2b|C,  5|C, 6|C,      lastPoint,     9, 4,    lastRelay },
    { 9, 5,  2a|C, 2b|C,  5|S,           lastPoint,     9, 5,    lastRelay },
} ;

const int nStreets = sizeof( accessories ) / sizeof( accessories[0][0] ) / nPointsPerStreet - 1 ; // calculate amount of streets, depending on the size of the table above

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

const int pcfAddress = 0x21;

Weistra pwmController( pwmPin1, pwmPin2, 50, 100 ) ;

DebounceClass buttons

#ifdef DEBUG
    #define debug Serial
#else
SoftwareSerial debug(9,10) ;
#include "src/XpressNetMaster.h"
XpressNetMasterClass    Xnet ;
#endif

EventHandler program( 0x0000, 0x7FFF, 0x50 ) ;


/* things to do:

PCB stuff
- SEPERATE PCB FOR CURRENT SENSING
- SEPERATE PCB FOR MOSFET CONTROLLING
- MAKE NEW BRANCH OF FunctionBloX 


code stuff
- get eventhandler to work   (HALFWAY THERE)
  V record/play PWM movements
  V record/play servos
  - record/play relays() propably works)
  - record input pins (not tested, may work already)
  - use separate channels
  - reserve 2 buttons (7 & 8) for train movements (ALSO, V2 HAS POTENTIOMETERS, DISCARD THIS?)
    (actually not needed maus is still needed to control IO, so might
    as well drive with it)

- different mode (#IFDEF OTHER MODE?)
   Use inputs to correctly control servos & relays

NOTE:
-  may just Function Blox for this?
  unused inputs should perhaps serve as feedback automatically?
  or make separate feedback objects without outgoing or ingoing links?

Make default Function Blox programs for different modi.
- let multimaus control relay & servos via FunctionBlox

FUNCTION BLOX STUFF:
- add Xnet methods
    RECEIVING
      loco speed (fixed address) (analog signal can be mapped and repurpose'd)
      loco function( fixed address) (digital out)
      point (fixed address)
    
    TRANSMITTING
      loco speed( variable address, variable speed) // can always fix address with a constant
      loco function( variable address, fixed state, F0- F10 )
      point( fixed or variable address?, state)


- add PWM controller
- add Stepper motor (with homing?) 3 inputs max, think of something.
  teachin-able with multimaus
- direct maus to stepper control
- direct maus to servo control?
- potentiometers to servo/stepper
- need an analog latch block. This would allow me to connect analog input to several servo's with one at the time.
  perhaps latch the servo's/steppers themselfes.


- may want to alter the stop button to double act as the record button by holding it for 2 seconds
  get code from hand controller

V test MCP inputs
- test PCF output (relais)
V test servo motors
V get XpressNet control functions to work
V test pwm controller working well in one try! 2 pin control.
V add shorcircuit code to weistra
V make motors configurable
- solder motor 1

BACKLOG
- add acceleration/decceleration to weistra 
*/

uint8_t relays ;
uint8_t relaysPrev ;
int8_t  speed = 0 ;
bool accelerating = true ;

const int mcpPins[] = {  // Software correction for physical mcp23017 pins
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
    uint16_t IO ;
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
    int8 speedActual = lookUpSpeed( Speed & 0b00011111 ) ;
    speedActual = map( speedActual, 0, 28, 0, SPEED_MAX ) ;           // map 28 speedsteps to 100 for weistra control
    if( Speed & 0x80 ) speedActual = -speedActual ;
    pwmController.setSpeed( speedActual ) ;
    program.storeEvent( speedEvent, 123, speedActual ) ;
}

void notifyXNetLocoDrive128( uint16_t Address, uint8_t Speed )
{
    int8 speedActual = Speed & 0x7F ;
    int8 direction   = Speed >> 7 ;
    
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

void notifyXNetgiveLocoInfo(uint8_t UserOps, uint16_t Address) // WHAT DOES THIS DO?
{
    #ifndef DEBUG
    Xnet.SetLocoInfo(UserOps, 0x00, 0x00, 0x00); //UserOps,Speed,F0,F1
    #endif
}

void notifyXNetgiveLocoFunc(uint8_t UserOps, uint16_t Address) // WHAT DOES THIS DO?
{
    #ifndef DEBUG
    Xnet.SetFktStatus(UserOps, 0x00, 0x00); //Fkt4, Fkt5
    #endif
}

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
    debug.begin(9600) ;
    debug.println("PWM controller booted") ;
    pinMode(13,OUTPUT) ;    
    initIO() ;
}

void loop()
{

    #ifndef DEBUG
    Xnet.update() ;
    #endif
    pwmController.update() ;
    debounceInputs() ;
    // processButtons() ; obsolete
    sweepServos() ;
    updateRelay() ;
    // program.update() ; not yet in use
}



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
                    state = setRoute ;
                    goto indexFound ;
                }
            }

            state = getFirstButton ;        
        indexFound:
            break ;

        case setRoute:
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
                    printNumber_("point set: ", address ) ;
                    printNumberln(": ", pointState ) ;
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
                    printNumberln("relay set: ", relay ) ;
                }
            break ;

        case waitDepature:
            if(1) state = waitArrival ; // wait for train to start moving
            break ;

        case waitArrival:
            if(1) state = getFirstButton ;

            // release control panel again.
            break ;
        }
    }
} 