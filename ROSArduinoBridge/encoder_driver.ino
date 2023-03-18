/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }

  void initEncoder(){
    
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;
        
	enc_last <<=2; //shift previous state two places
   #if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_NANO
      enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
   #elif defined(ARDUINO_AVR_MEGA1280) || defined(ARDUINO_AVR_MEGA2560)
	    enc_last |= (LEFT_PIN & (3 << 2)) >> 2; //read the current state into lowest 2 bits
   #endif 
  
  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
      static uint8_t enc_last=0;
          	
	enc_last <<=2; //shift previous state two places
	
	 #if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_NANO
    enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
    
   #elif defined(ARDUINO_AVR_MEGA1280) || defined(ARDUINO_AVR_MEGA2560)
      //enc_last |= (RIGHT_PIN & (3 << 6)) >> 6; //read the current state into lowest 2 bits
   #endif 
	
	
  
  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
   
   
  }

  ISR (PCINT0_vect){
    static uint8_t enc_last=0;
            
    enc_last <<=2; //shift previous state two places
  
    #if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_NANO
      //enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
    
    #elif defined(ARDUINO_AVR_MEGA1280) || defined(ARDUINO_AVR_MEGA2560)
      enc_last |= (RIGHT_PIN & (3 << 6)) >> 6; //read the current state into lowest 2 bits
    #endif 
      
    right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }

void initEncoder(){
  #if defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_NANO
  
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
    
    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);
    
    // tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
    
    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE1) | (1 << PCIE2);  
    
  #elif defined(ARDUINO_AVR_MEGA1280) || defined(ARDUINO_AVR_MEGA2560)
    
    //set as inputs
    LEFT_DDR &= ~(1<<LEFT_ENC_PIN_A);
    LEFT_DDR &= ~(1<<LEFT_ENC_PIN_B);
    RIGHT_DDR &= ~(1<<RIGHT_ENC_PIN_A);
    RIGHT_DDR &= ~(1<<RIGHT_ENC_PIN_B);
    
    //enable pull up resistors
    LEFT_PORT |= (1<<LEFT_ENC_PIN_A);
    LEFT_PORT |= (1<<LEFT_ENC_PIN_B);
    
    RIGHT_PORT |= (1<<RIGHT_ENC_PIN_A);
    RIGHT_PORT |= (1<<RIGHT_ENC_PIN_B);
    
    // tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);

    PCMSK0 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
    
    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE0)| (1 << PCIE1) | (1 << PCIE2);

  #endif     
  }


#elif defined(ARDUINO_SIMPEL_ODO)

  extern boolean getLeftDirection(void);
  extern boolean getRightDirection(void);

  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;

  void initEncoder(){
    pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightInterrupt, CHANGE);
  }

   /* Interrupt routine for LEFT encoder, taking care of actual counting */
  void leftInterrupt(){
    if(getLeftDirection()){
      left_enc_pos ++;
    } else {
      left_enc_pos --;
    }
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
 void rightInterrupt(){
    if(getRightDirection()){
      right_enc_pos ++;
    } else {
      right_enc_pos --;
    }
 }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }

  
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
