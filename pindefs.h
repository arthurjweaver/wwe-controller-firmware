// ---------- pindefs.h ----------
// See https://www.arduino.cc/reference/en/language/functions/digital-io/digitalwrite/
// which is replaced by digitalWriteDirect() below

#ifndef PINS_DEFINED
#define PINS_DEFINED

//#define SPI_SS_PIN 4  // SD card

// Stepper motor control
#ifdef ENABLE_STEPPER
#define MOTOR_STEP_PIN 5  // 15/9
#define MOTOR_DIR_PIN 6   // 15/9
#define MOTOR_ENBL_PIN 8  // 15/9
#endif

// Status LED pin
//#define myLED 13          // 3/6

// Pins for manual control of stepper motor, shorting contactor, and manual mode
#define PIN_LOW_FORCE_FURL 22    // 3/6
#define PIN_LOW_FORCE_UNFURL 23  // 15/9
#define PIN_LOW_SHORT_SC 24      // 15/9
#define PIN_LOW_UNSHORT_SC 25    // 15/9
#define PIN_LOW_MANUAL_MODE 26   // 15/9

#define DUMP_IGBT_DRV_PIN 28     // 15/9, dump load control pin
#define INVERT_DUMP_DRIVE        // if defined, invert sense of dump load driver - see furlctl.ino
#ifdef INVERT_DUMP_DRIVE
#define DUMP_LOAD_ON LOW         // if processor connected to base of P-Channel FET
#define DUMP_LOAD_OFF HIGH
#else
#define DUMP_LOAD_ON HIGH        // if processor output connected directly to optoisolator anode
#define DUMP_LOAD_OFF LOW
#endif

// Furl/Unfurl control pins for a linear actuator motor - used by furlctl(), see furlctl.ino
// Not used with stepper motor
#define UNFURL_CTL_PIN 29             // 15/9
#define FURL_CTL_PIN 30               // 15/9

// Stepper motor control pin
#define STEPPER_MOTOR_INT_PIN 31      // 15/9

// RS485 control pin
#define RS485_ENBL_PIN 32             // 

// Shorting contactor, board temp, reed switch control pins
#define SC_CTL_PIN 33                 // 15/9
#define ONE_WIRE_TEMP_PIN 34          // controller logic board temp
#define REED_SWITCH_PIN 37            //

// Status LED pins
#define REED_SWITCH_STATE_LED_PIN 38
#define DUMP_LOAD_ACTIVE_LED_PIN 42   // 15/9
#define SC_ACTIVE_LED_PIN 44          // 15/9
#define FURL_STATE_1_LED_PIN 46       // 15/9
#define FURL_STATE_2_LED_PIN 48       // 15/9
#define TIMER_LOOP_LED_PIN 50         // 15/9
#define MAIN_LOOP_LED_PIN 52          // 3/6

// Logic Analyzer connection pins
#define STATUS1_PIN_1 39              // 15/9
#define STATUS1_PIN_2 41              // 15/9
#define STATUS1_PIN_4 43              // 3/6
#define STATUS1_PIN_8 45              // 15/9
#define STATUS2_PIN_1 47              // 15/9
#define STATUS2_PIN_2 49              // 15/9
#define STATUS2_PIN_4 51              // 15/9
#define STATUS2_PIN_8 53              // 15/9


// Faster version of digitalWrite() - use everywhere instead
inline void digitalWriteDirect(int pin, boolean val){
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

inline int digitalReadDirect(int pin){
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}


// Initialize Arduino digital pins.
//
// If a pin is set as an OUTPUT, writeDigital(pin,HIGH) = 5V (or 3.3V on 3.3V boards), LOW = 0V (ground).
//
// If a pin is set as an INPUT, writeDigital(pin,HIGH) enables the internal pullup on the pin, LOW disables it.
// It is recommended to set pinMode() to INPUT_PULLUP to enable the internal pull-up resistor. See the Digital Pins tutorial for more information.
//
// If you do not set the pinMode() to OUTPUT, and connect an LED to a pin, when calling digitalWrite(HIGH), the LED may appear dim. 
// Without explicitly setting pinMode(), digitalWrite() will have enabled the internal pull-up resistor, which acts like a large current-limiting resistor.

void initPins() {

  //pinMode(SPI_SS_PIN, OUTPUT);                 // SD card

  // Control pins
  pinMode(DUMP_IGBT_DRV_PIN, OUTPUT);          // dump load
  pinMode(FURL_CTL_PIN, OUTPUT);               // FURL 
  pinMode(UNFURL_CTL_PIN, OUTPUT);             // UNFURL
  pinMode(SC_CTL_PIN, OUTPUT);                 // shorting contactor
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);      // reed switch

  // Status LED pins
  //pinMode(myLED, OUTPUT);                      // health check
  pinMode(MAIN_LOOP_LED_PIN, OUTPUT);          // loop() health 
  pinMode(TIMER_LOOP_LED_PIN, OUTPUT);         // timer health
  pinMode(FURL_STATE_1_LED_PIN, OUTPUT);       // state 1 = FURLING
  pinMode(FURL_STATE_2_LED_PIN, OUTPUT);       // state 2 = FURLED
  pinMode(SC_ACTIVE_LED_PIN, OUTPUT);          // shorting contactor state
  pinMode(DUMP_LOAD_ACTIVE_LED_PIN, OUTPUT);   // dump load state
  pinMode(REED_SWITCH_STATE_LED_PIN, OUTPUT);  // reed switch state
  
  pinMode(7, OUTPUT);                          // ???
  pinMode(MOTOR_ENBL_PIN, OUTPUT);             // defined above as pin 8

  // Logic Analyzer status
  pinMode(STATUS1_PIN_1, OUTPUT);
  pinMode(STATUS1_PIN_2, OUTPUT);
  pinMode(STATUS1_PIN_4, OUTPUT);
  pinMode(STATUS1_PIN_8, OUTPUT);   
  pinMode(STATUS2_PIN_1, OUTPUT);
  pinMode(STATUS2_PIN_2, OUTPUT);
  pinMode(STATUS2_PIN_4, OUTPUT);
  pinMode(STATUS2_PIN_8, OUTPUT);
  

  pinMode(RS485_ENBL_PIN, OUTPUT);        // activate RS485 driver for output...
  digitalWriteDirect(RS485_ENBL_PIN, 0);  // set it low to disable driver
  
#ifdef ENABLE_STEPPER
  pinMode(STEPPER_MOTOR_INT_PIN, OUTPUT);
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_ENBL_PIN, OUTPUT);
  pinMode(DAC0, OUTPUT);
  pinMode(DAC1, OUTPUT);
  pinMode(PIN_LOW_FORCE_FURL, INPUT_PULLUP);
  pinMode(PIN_LOW_FORCE_UNFURL, INPUT_PULLUP);
  pinMode(PIN_LOW_SHORT_SC, INPUT_PULLUP);
  pinMode(PIN_LOW_UNSHORT_SC, INPUT_PULLUP);
  pinMode(PIN_LOW_MANUAL_MODE, INPUT_PULLUP);
#endif

}  // END initPins()


// Macro to check the value of a bit in a word
#define CHECK_BIT(var,pos) !!((var) & (1<<(pos)))


// Use these functions to write status code to output pins
void writeStatus1(int val) {
  digitalWriteDirect(STATUS1_PIN_1, CHECK_BIT(val, 0));  
  digitalWriteDirect(STATUS1_PIN_2, CHECK_BIT(val, 1));
  digitalWriteDirect(STATUS1_PIN_4, CHECK_BIT(val, 2));
  digitalWriteDirect(STATUS1_PIN_8, CHECK_BIT(val, 3));
}

// This function is called in stepper.h if SHOW_MOTOR_STATUS is defined
void writeStatus2(int val) {
  digitalWriteDirect(STATUS2_PIN_1, CHECK_BIT(val, 0));  
  digitalWriteDirect(STATUS2_PIN_2, CHECK_BIT(val, 1));
  digitalWriteDirect(STATUS2_PIN_4, CHECK_BIT(val, 2));
  digitalWriteDirect(STATUS2_PIN_8, CHECK_BIT(val, 3));
}

#endif
