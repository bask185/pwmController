#include <Arduino.h>

const int       C       = 0x8000 ;          // CURVED
const int       S       = 0x0000 ;          // STRAIGHT
const int       last    = 10000 ;   // must be in the end of every line in table to flag that the last point is set.

const int       nPointsPerStreet = 12 ;

const uint16_t Routes[][nPointsPerStreet+3] =
{
//   blokken   wissels + standen        laatste wissel  extra block leds
    { 1, 6,   3|S,                          last,         1, 6,    last }, 
    { 1, 8,   3|S,   1|S,                   last,         1, 6,    last },
    { 2, 6,   3|C,   4|C,                   last,         2, 6,    last },
    { 2, 7,   4|S,                          last,         2, 7,    last },
    { 2, 8,   1|S, 3|C, 4|C,                last,         2, 6,    last },
    { 2, 9,   4|S, 2|S, 7|S,                last,         2, 7, 8, last },
    { 6, 8,   1|S,                          last,         6,       last },
    { 7, 9,   2|S, 7|S,                     last,         7, 9,    last },
    { 8, 3,   1|C, 2|C, 7|C, 5|S, 6|S,      last,         3,       last },
    { 8, 4,   1|C, 2|C, 7|C, 5|S, 6|C,      last,         4,       last },
    { 8, 5,   1|C, 2|C, 7|C, 5|C,           last,         5,       last },
    { 9, 3,        2|S, 7|C, 5|S, 6|S,      last,         8, 3,    last },
    { 9, 4,        2|S, 7|C, 5|S, 6|C,      last,         8, 4,    last },
    { 9, 5,        2|S, 7|C, 5|C,           last,         8, 5,    last },
    { 7, 8,   1|C, 7|S,                     last,         7,       last },
} ;

struct
{
    uint8_t     firstButton ;
    uint8_t     secondButton ;
    uint16_t    point[nPointsPerStreet] ;
    uint8_t     extraButtons[5] ;
} Route;