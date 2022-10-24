
#define RS485DIR 2
#define servoPin1 3
#define servoPin2 4
#define servoPin3 5
#define servoPin4 6
#define servoPin5 7
#define servoPin6 8
#define servoPin7 9
#define servoPin8 10
#define statusLed 13
#define pinNumber 14
#define pwmPin1 A0
#define pwmPin2 A1
#define shortCircuit A2
#define swPin1 0
#define swPin2 1
#define swPin3 2
#define swPin4 3
#define swPin5 4
#define swPin6 5
#define swPin7 6
#define swPin8 7
#define swPin9 8
#define swPin10 9
#define swPin11 10
#define swPin12 11
#define swPin13 12
#define swPin14 13
#define swPin15 14
#define swPin16 15

extern void initIO(void);
extern void mcpWrite(unsigned char pin, unsigned char state);
extern unsigned char mcpRead(unsigned char pin);