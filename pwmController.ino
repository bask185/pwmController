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
#include "src/mcp23017.h"
#include "Wire.h"

#define mcpAddress 0x20
#define portA	    0x12
#define portB	    0x13
#define iodirRegA	0x00
#define iodirRegB	0x01
#define pullUpRegA	0x0C
#define pullUpRegB	0x0D

const int F1_F4 = 0 ;
const int F5_F8 = 0x80 ;
const int F11 = -3 ; // degrees at the time
const int F12 =  3 ;
const int SPEED_MAX = 50 ; 
uint8_t speedFeedback ;
const int decouplerPin = 7 ;

const int nButtons = 9 ;

EventHandler program( 0x0000, 0x7FFF, 0x50 ) ;

enum events
{
    relayEvent ,
    servoEvent,
    speedEvent,
    ledEvent,
    buttonEvent,
} ;

const int nPointsPerStreet = 8 ;

const int       C = 0x8000 ;          // CURVED
const int       S = 0x0000 ;          // STRAIGHT
const int       last = 10000 ;   // must be in the end of every line in table to flag that the last point is set.
const int       STRAIGHT = 1 ;
const int       CURVED = 2 ;
const int       OFF = 0 ;


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
    { 8, 3,   1|C, 2|C, 7|C, 5|S, 6|S,      last,     3,       last },
    { 8, 4,   1|C, 2|C, 7|C, 5|S, 6|C,      last,     4,       last },
    { 8, 5,   1|C, 2|C, 7|C, 5|C,           last,     5,       last },
    { 9, 3,        2|S, 7|C, 5|S, 6|S,      last,     8, 3,    last },
    { 9, 4,        2|S, 7|C, 5|S, 6|C,      last,     8, 4,    last },
    { 9, 5,        2|S, 7|C, 5|C,           last,     8, 5,    last },
    { 7, 8,   1|C, 7|S,                     last,     7,       last },
} ;

const int nStreets = 15 ;  // 1   2   3   4   5   6   7

const uint8_t buttonLed[] = {255,  7,  6,  5,  4,  3,  2,  1, 0, 8} ;
const uint8_t  pointLed[] = {255, 10, 10,  9,  9, 12, 11, 13 } ;
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
    freeRoute,
} ;


uint8 state = getFirstButton ;

uint8 firstButton, secondButton ;
uint8 index = 0 ;
uint8 street ;
int8_t speedActual ;

const uint32_t decouplerTime    = 3500 ; // decoupler time in ms
const uint32_t recordPressTime  = 3500 ;

const int pcfAddress = 0x21;

Weistra pwmController( pwmPin1, pwmPin2, 50, 100 ) ;

#ifdef DEBUG
    #define debug Serial
#else
SoftwareSerial debug(9,10) ;
#include "src/XpressNetMaster.h"
XpressNetMasterClass    Xnet ;
#endif

uint8_t relaysPrev ;
uint8_t relays ;
int8_t  speed = 0 ;
bool    accelerating = true ;
bool    decouplerSet = false ;
bool    programState = false ;

uint8_t buttonState[16] ;

uint8 debounceIndex = 0 ;
Debounce programButton( 255 ) ;


const int mcpPins[] = {  // Software correction for physical mcp23017 pins
  9, 255, 255, 255, 255, 255, 15, 16, 8, 7, 6, 5, 4, 3, 2, 1
} ;  // 10 = decoupler, 11 = play/record, 255 is led, rest is button

void debounceInputs()
{
    uint16_t IO ;
    static uint16_t prevIO = 0xFFFF ;
    REPEAT_MS( 50 )
    {
        Wire.beginTransmission( mcpAddress ) ;
        Wire.write( portA ) ;
        Wire.endTransmission() ;
        Wire.requestFrom( mcpAddress, 2 ) ;
        IO = (Wire.read() << 8) | Wire.read() ;  

        programButton.debounce( programState ) ;
    }
    END_REPEAT

    for( int i = 0 ; i < 11 ; i ++ )
    {
        buttonState[i] = HIGH ;
    }

    if( IO != prevIO )
    {
        for( int i = 0 ; i < 16 ; i ++ )
        {
            if((    IO & (1<<i)) 
            != (prevIO & (1<<i)) )
            {
                uint8_t pin = mcpPins[i] ;  

                if( !(IO & (1<<i)) )
                {
                    //if( pin == 255 ) break ;

                    if( pin == 15 )
                    {
                        decouplerSet = 1 ;
                        printNumberln("FALLING ", pin) ;
                        break ; // decoupler set
                    }
                    if( pin == 16 )
                    {
                        programState = 0 ;
                        printNumberln("FALLING ", pin) ;
                        break ; 
                    }
                    
                    buttonState[pin-1] = FALLING ;
                    program.storeEvent( buttonEvent , pin-1 , FALLING ) ;
                    printNumberln(F("recorded button (FALLING): "),pin-1);
                    printNumberln("FALLING ", pin) ;
                    prevIO = IO ;
                }
                else
                {
                    if( pin == 11 )
                    {
                        programState = 1 ;
                    }
                    programState = 1 ;
                    pin = mcpPins[i] ; 
                    printNumberln("RISING ", pin) ;
                    buttonState[mcpPins[i]-1] = RISING ;
                }
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

            if(      ioNumber <=  8 ) { setLed( pointLed[ioNumber], state + 1 ) ;
                                        setServo( ioNumber - 1 , state ) ;  }                   //  1 <->  8
            else if( ioNumber <= 18 ) { bitWrite( relays, ioNumber-11, state ) ; }              // 11 <-> 18 .. ADD BUTTON LEDS HERE
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
        case speedEvent:    pwmController.setSpeed( data) ;      
                            speedActual = data ;                 break ;
        case servoEvent:    setServo( address, data ) ;          break ;
        case relayEvent:    bitWrite( relays, address, data ) ;  break ;     
        case ledEvent:      setLed( address, data ) ;            break ;
        case buttonEvent:   buttonState[address] = FALLING ;        
                            printNumberln(F("buttonEvent"), address ) ;
                            break ;
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

void updateDecoupler()
{
    static uint32_t prevTime = 0 ;

    if( decouplerSet == true )
    {   decouplerSet = false ;

        prevTime = millis() ; // 2.5s
        setServo( decouplerPin, 1 ) ;         // move decoupler up
        program.storeEvent( servoEvent,  decouplerPin, 1 ) ; 
        printNumberln(F("decoupler up"), 1);
    }
    else if( millis() - prevTime >= decouplerTime && getServo(decouplerPin) == true ) // if decoupler is up and time has expired
    {
        setServo( decouplerPin, 0 ) ;         // move decoupler down
        program.storeEvent( servoEvent,  decouplerPin, 0 ) ; 
        printNumberln(F("decoupler down"), 1); 
    }
}

void programCommand()
{
    static uint32_t prevTime ;
    static uint8_t  disardRising ;
    uint8_t state = programButton.getState() ;

    if( millis() < 1000 ) {program.stopRecording() ; return ;}

    if( state == FALLING )
    {
        disardRising = false ;
        prevTime = millis() ;
    }
    if( state == LOW )
    {
        if( millis() - prevTime >= recordPressTime && disardRising == false ) // long press
        {
            disardRising = true ;
            printNumberln(F("start recording "),1) ;
            if( program.getState() == idle ) program.startRecording() ;
        }
    }

    if( state == RISING && disardRising == false ) // release after short press
    { 
        uint8_t programState = program.getState() ;
        if( programState == recording ) { program.stopRecording() ; printNumberln(F("stop Recording "),1) ; return ; }
        if( programState == playing   ) { program.stopPlaying() ;   printNumberln(F("stop Playing "),2) ;   return ; }
        if( programState == idle      ) { program.startPlaying() ;  printNumberln(F("start Playing "),3) ;  return ; }
    } 
}


// MOVE ME OUT OF HERE INTO THE .INO
void setLed( uint8_t pin, uint8_t state ) // CURVED, STRAIGHT, OFF
{
    uint8_t val ;
    uint8_t port  =  portA ;
    uint8_t ioDir = iodirRegA ;

    if( pin >= 8 )
    {
        pin  -= 8 ;
        port  =  portB ;
        ioDir = iodirRegB ;
    }

    if( state > OFF ) // STRAIGHT OR CURVED
    {
        val = readRegister( ioDir ) ;  // set pin to output
        bitWrite( val, pin, 0 ) ;
        writeRegister( ioDir, val ) ;

        val = readRegister( port ) ;
        if( state == CURVED ) { bitWrite( val, pin, 1 ) ; } // curved
        else                  { bitWrite( val, pin, 0 ) ; } // straight
        writeRegister( port, val ) ;
    }
    else    // OFF
    {
        val = readRegister( ioDir ) ;  // set pin to input (kills the leds)
        bitWrite( val, pin, 1 ) ;
        writeRegister( ioDir, val ) ;
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
    Wire.write(0b11000001);
    Wire.endTransmission();

    Wire.beginTransmission(mcpAddress);
    Wire.write(pullUpRegA);
    Wire.write(0b11111111);
    Wire.write(0b11000001);
    Wire.endTransmission();

    debug.begin( 9600 ) ;
    debug.println("PWM controller booted") ;

    programButton.debounce(1);
    programButton.debounce(1);
}


void loop()
{
    #ifndef DEBUG
    Xnet.update() ;
    #endif
    pwmController.update() ;
    debounceInputs() ;
    program.update() ;
    runNx() ;
    sweepServos() ;
    updateRelay() ;
    updateDecoupler() ;
    programCommand() ; 
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
        firstButton = 0 ;
        secondButton = 0 ;
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
            if( buttonState[i] == RISING )
            { 
                state = freeRoute ;
                printNumberln("released ", firstButton ) ;
                break ;
            }
            if( buttonState[i] == FALLING )
            {
                secondButton = i+1 ;
                printNumberln("secondButton ", secondButton ) ;
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
                printNumberln("street found ", street ) ;
                for( int i = 1 ; i < 8 ; i ++ )
                {
                    setLed( pointLed[i], OFF ) ;
                }
                setLed( buttonLed[firstButton], STRAIGHT ) ;
                setLed( buttonLed[secondButton], STRAIGHT ) ;
                index = 2 ;                 // reset index before setting new street
                state = setRoute ;
                return ;
            }
        }

        state = freeRoute ;
        printNumberln("no index found ", 0) ;
        break ;

    case setRoute:
        REPEAT_MS( 333 )
        {
            point = accessories[street][index++]  ;

            if( point == last )
            {
                state = setRelays ;
                break ;
            }
            else
            {
                uint16_t address = point & 0x03FF ;
                pointState = point >> 15 ;
                setServo( address-1, pointState ) ;
                setLed( pointLed[address], pointState+1 ) ;
                printNumberln("point set", address) ;
            }
        }
        END_REPEAT
        break ;

    case setRelays:
        relays = 0 ;
        while( state == setRelays )
        {
            relay = accessories[street][index++] ;
            if( relay == last )
            {
                state = waitDepature ;
                break ;
            }
            else
            {
                relays |= (1 << (relay-1) ) ;
                if( relay == 6 ) setLed( buttonLed[6], STRAIGHT ) ;
                if( relay == 7 ) setLed( buttonLed[7], STRAIGHT ) ;
                printNumberln("relay set: ", relay ) ;
            }
        }
        break ;

    case waitDepature:
    #ifndef DEBUG
        if( speedActual != 0 ) state = waitArrival ;
    #else
        state = freeRoute ;
    #endif
        break ;

    case waitArrival:
        if( speedActual == 0 ) state = freeRoute ;
        break ;

    case freeRoute:
        for( int i = 1 ; i < 10 ; i ++ )
        {
            setLed( buttonLed[i], OFF ) ;
            if( i < 8 )
            {
               uint8_t state = getServo( i ) + 1 ;
               setLed( pointLed[i], state ) ;
            }
        }
        //if( buttonState )
        state = getFirstButton ;
        printNumberln("RESETTING ", state) ;
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
        // fill me in ;
    }
}
#endif