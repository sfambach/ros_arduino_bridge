/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */

   
#ifdef ARDUINO_ENC_COUNTER
  

  #if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_NANO

    //below can be changed, but should be PORTD pins; 
    //otherwise additional changes in the code are required
    #define LEFT_ENC_PIN_A PD2  //pin 2
    #define LEFT_ENC_PIN_B PD3  //pin 3

     //below can be changed, but should be PORTC pins
    #define RIGHT_ENC_PIN_A PC0  //pin A0
    #define RIGHT_ENC_PIN_B PC1   //pin A1

  #elif defined(ARDUINO_AVR_MEGA1280) || defined(ARDUINO_AVR_MEGA2560)

    #define LEFT_DDR  DDRK
    #define LEFT_PORT PORTK
    #define LEFT_PIN  PINK
    #define LEFT_ENC_PIN_A PK2  //pin A10
    #define LEFT_ENC_PIN_B PK3  //pin A11

    #define RIGHT_DDR  DDRB
    #define RIGHT_PORT PORTB
    #define RIGHT_PIN  PINB
    #define RIGHT_ENC_PIN_A PB7  // pin 14
    #define RIGHT_ENC_PIN_B PB6  // pin 15


  #else
    #error undefined board :(
  #endif

#elif defined(ARDUINO_SIMPEL_ODO)
/*simple odometrie, can indicate speed but not direction
* Direction must be taken from the motor drive.
* This will not work if the wheels are spinning manually without the motor*/ 

   #define LEFT_ENC_PIN_A 2  //pin 2
   #define RIGHT_ENC_PIN_A 3  // pin 3

#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
