#include "controlPanel.h"
#include "Wire.h"

static uint16_t GPIO ;
static uint16_t GPIO_DIR ;

#define portA        0x12
#define portB        0x13
#define iodirRegA    0x00
#define pullUpRegA   0x0C

void setGpio( uint8_t bit, uint8_t state )
{
    if( state ) GPIO |=  (1 << bit) ;
    else        GPIO &= ~(1 << bit) ;
}

// what must we do? 
// keep reading inputs,
// switch from internal pullup input to output low
// force GPIO to internal pullup before a read.
void updateGpio()
{
    Wire.beginTransmission( 0x20 ) ;
    Wire.write( iodirRegA ) ;           // set pins to input
    Wire.write( 0xFF  ) ;
    Wire.write( 0xFF  ) ;
    Wire.endTransission() ;

    Wire.beginTransmission( 0x20 ) ;
    Wire.write( pullUpRegA ) ;          // set pins to input_pullup
    Wire.write( 0xFF  ) ;
    Wire.write( 0xFF  ) ;
    Wire.endTransission() ;

    Wire.beginTransmission( 0x20 ) ;    // fetch inputs
    


}



unsigned char mcpRead(unsigned char pin) {
    unsigned char address = 0x20, port, IO;
    address += (pin / 16);                // select address

    if((pin % 16) < 8)    port = portA;    // select port
    else                port = portB;
    
    pin %= 8 ;

    Wire.beginTransmission(address);
    Wire.write(port);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);
    IO = Wire.read();

    if(IO & (1 << pin))    return 1;
    else                return 0;
}
