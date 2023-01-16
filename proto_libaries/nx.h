#include <Arduino.h>

const int FIRST_BUTTON  = 1 ;
const int SECOND_BUTTON = 2 ;

// INITIALIZATION  _t _relaisPresent, uint8_t _freeRouteOnTrain, uint8_t _directionMatters )
/**
 * @brief initializes the library
 * @param relaisPresent analog layouts can make use of relais for the begin and end tracks and tracks in between
 * @param freeRouteOnTrain frees up the route after the train has stopped moving, otherwise route is freed after being set
 * @param directionMatters if set, one must use separate routes for each directon of travel, otherwise buttons work ambiguous
 */
extern void NxBegin( uint8_t, uint8_t, uint8_t ) ;


/**
 * @brief stores route to I2C EEPROM
 * @param pointer point to passed array
 * @param length  length of data to store
 * */
extern void storeRoutes( uint8_t *, uint16_t ) ;



// CONTROL FUNCTIONS
/**
 * @brief Pass the first and second pressed buttons
 * @param id first or second button?
 * @param val number of button
 */
extern void setNxButton( uint8_t, uint8_t ) ;



/**
 * @brief Pass the speed to the NX handler to free up a route
 * @param speed speed of loco
 */
extern void setNxSpeed( int8_t ) ;



// ROUND ROBIN TASK
extern void runNx( ) ;



// CALLBACK FUNCTIONS
/**
 * @brief is called by library to set a turnout
 * @param address of turnout
 * @param state of turnout
 */
extern void setNxTurnout( uint16_t, uint8_t ) __attribute__((weak)) ;


/**
 * @brief called by library to set a relay
 * @param address of relay
 * @param state of relay
 */
extern void setNxRelay(   uint16_t, uint8_t ) __attribute__((weak)) ;

/**
 * @brief called by library if the route is laid in
*/
extern void routeSet( ) __attribute__((weak)) ;


/**
 * @brief called by library if the route is freed up
*/
extern void routeFreed( ) __attribute__((weak)) ;


/**
 * @brief called by library if data from EEPROM is corrupted
*/
extern void invalidData( ) __attribute__((weak)) ;