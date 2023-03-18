/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
  
#elif defined ARDUINO_MOTOR_SHIELD_V20
  /* needs https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library */
  #include <Adafruit_MotorShield.h>
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();

  #ifndef AFMS_LEFT_MOTOR 
    #define AFMS_LEFT_MOTOR 1
  #endif

  #ifndef AFMS_RIGHT_MOTOR 
    #define AFMS_RIGHT_MOTOR 4
  #endif
  
  Adafruit_DCMotor *leftMotor  = AFMS.getMotor(AFMS_LEFT_MOTOR);
  Adafruit_DCMotor *rightMotor = AFMS.getMotor(AFMS_RIGHT_MOTOR);

#else
  #error Motor driver was not selected
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
