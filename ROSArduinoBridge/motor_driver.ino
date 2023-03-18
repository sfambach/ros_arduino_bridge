/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }


#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }


#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
    
  }

  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == LEFT) { 
      if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
    }
  }

#elif defined ARDUINO_MOTOR_SHIELD_V20
/**********************************************************************************
 * Motor shield v2 implementation
 **********************************************************************************/
  volatile boolean leftDirection = true;
  volatile boolean rightDirection = true;
 
  void initMotorController() {
     if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
      // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
     }
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    // check if the motor should drive backwards
    if (spd < 0)
    {
      
      spd = -spd;
      reverse = 1;
    }

    // shrink to max 255
    if (spd > 255)
      spd = 255;

    Adafruit_DCMotor *curMotor;

    
    if (i == LEFT) { 
      curMotor = leftMotor;
      leftDirection = !reverse;
      
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      curMotor = rightMotor;
      rightDirection = !reverse;
    }

    curMotor->setSpeed(spd);
    
    if (reverse == 0) {
        curMotor->run(FORWARD); 
        
    }
    else if (reverse == 1) { 
        curMotor->run(BACKWARD);   
    }
  }

  boolean getLeftDirection(){return leftDirection;}
  boolean getRightDirection(){return rightDirection;}
  

#else
  #error A motor driver must be selected!
#endif

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#endif // USE_BASE
