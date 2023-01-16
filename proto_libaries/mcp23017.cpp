#include "mcp23017.h"
#include <Wire.h>

const int mcpAddress = 0x20 ;

void writeRegister( uint8_t reg, uint8_t val )
{
    Wire.beginTransmission( mcpAddress ) ;
    Wire.write( reg ) ;
    Wire.write( val ) ;
    Wire.endTransmission() ;
}

uint8_t readRegister( uint8_t reg )
{
    Wire.beginTransmission( mcpAddress ) ;
    Wire.write( reg ) ;
    Wire.endTransmission() ;
    
    Wire.requestFrom( mcpAddress, 1 ) ;
    return Wire.read( ) ;
}