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



void setup()
{
    debug.begin( 9600 ) ;
    debug.println("PWM controller booted") ;
    nx.begin(0,0);
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
/*
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
*/
void notifyXNetTrnt(uint16_t Address, uint8_t data)
{
    if( bitRead( data, 3 ) == 1 )
    {
        // fill me in
    }
}
#endif