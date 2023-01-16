#include <Arduino.h>

const int FIRST_BUTTON  = 1 ;
const int SECOND_BUTTON = 2 ;

extern void NxBegin( uint8_t, uint8_t, uint8_t ) ;
extern void setNxButton( uint8_t, uint8_t ) ;
extern void runNx() ;
extern void setNxSpeed( int8_t ) ;
extern void setNxTurnout( uint16_t, uint8_t ) __attribute__((weak)) ;
extern void setNxRelay(   uint16_t, uint8_t ) __attribute__((weak)) ;
extern void invalidData( ) __attribute__((weak)) ;