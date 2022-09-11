// ---------- wwe.ino ----------
// This is a sketch for a Weaver Wind Energy W2 wind turbine controller with an Arduino Due 
// (SAM DUE ARM Cortex-M3) microprocessor + Ethernet2 shield.

// W2 system components:
//   1. W2 wind turbine data are collected and processed directly by the controller logic board.
//   2. An Etesian wireless anemometer base unit is connected to the controller on Serial2.
//   3. Morningstar charge and diversion load controllers are connected to the controller via Modbus/RTU on Serial3.
//   4. A Nuvation battery management system (BMS) supplies data to the controller via Modbus/TCP (Ethernet).
//   5. An Outback Radian inverter data does NOT send data to the controller. 
//      Instead, it sends data to a system monitor web page via the Mate3s "Data Stream" system setting.

// This is the MAIN MODULE which calls:
//   setup()
//   loop()

// There are a few additional functions in this module:
//   void startTimer() - starts the INTERRUPT TIMER
//   boolean doFWUpdate() - updates firmware
//   void TC0_Handler() - apparently unused, left over from early dev?
//   void TC8_Handler() - apparently unused, left over from early dev?

// ***COMPILE NOTES***
//   To UPDATE THIS FIRMWARE, this program visits an "Update Server" and accesses a compiled binary file: "updatefw.bin".
//   That file is downloaded, written to FLASH1 at address 0xC0000 and started at that location. 
//   As such, the compiled wwe.ino CANNOT occupy a flash memory space greater than 0x40000 (256kB) in length.
//   This is 1/2 the usual maximum program size of 524288 bytes (512kB).
//
// To build this program to execute at 0x80000 with a size not exceeding 0x40000, EDIT:
//   %USERPROFILE%\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\variants\arduino_due_x\linker_scripts\gcc\flash.ld
//
// COMMENT OUT the original line and ADD two lines (also intentionally commented-out):
  /*rom (rx)    : ORIGIN = 0x00080000, LENGTH = 0x00080000*/ /* Flash, 512K */
  /*rom (rx)    : ORIGIN = 0x00080000, LENGTH = 0x00040000*/ /* Flash Bank 0, 256K */
  /*rom (rx)    : ORIGIN = 0x000C0000, LENGTH = 0x00040000*/ /* Flash Bank 1, 256K */
//
// Note the shorter LENGTH = 0x40000 for both ORIGIN's!
//   This is necessary because wwe.ino writes a bootloader program, updatefw.bin, to 0xC0000, which
//   must NOT overlap its own program space, as it would be if its LENGTH were 0x80000.
//   See https://stackoverflow.com/questions/47078293/atmel-sam3x-dual-bank-switching-not-working
// THEN...
//   ***UNCOMMENT the Flash Bank 0 line when creating a compiled binary for wwe.ino***, or
//   Uncomment the Flash Bank 1 line when creating a compiled binary for updatefw.ino
//
// Create a compiled binary file: 
//   Sketch --> Export compiled Binary
//
// Open a Cygwin64 Terminal window and copy compiled binary (and source) from the PC to Mac server:
//   cd /cygdrive/c/Users/SANDY/Documents/Arduino/wwe && scp * wwe@192.168.1.4:/Users/wwe/Sites/WWE/bin/firmware
//
// TO FORCE A FIRMWARE UPDATE *ANYTIME*:
//   1. Override Server? must be set to 0
//   2. change Update File to anything other than the firmware name saved in the server config.
//   3. click SUBMIT
//   4. WAIT Update Interval minutes for the next config update.
//      The server will see the discrepancy in the Update File name and will start a firmware update cycle.


// ***LIBRARY NOTES***
//   see https://www.arduino.cc/en/guide/libraries
//   Arduino libraries are managed in 3 different places: 
//     1. the IDE installation folder: C:\Program Files (x86)\Arduino\libraries
//     2. the core folder: C:\Users\wwe\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.10\libraries
//     3. the sketchbook libraries folder: ~\Documents\Arduino\libraries
//   The way libraries are chosen during compilation allows for updates to the distribution libraries. 
//   This means that placing a library your sketchbook libraries folder OVERRIDES the other libraries versions.
//   To ADD or UPDATE a library: Sketch --> Include Library --> Manage Libraries or Install .ZIP Library
//     If installing a .ZIP library, put the .ZIP file in Downloads or someplace OTHER than the installation folder
//     prior to doing "Install .ZIP Library"
//   To DELETE a library: Open terminal window --> cd to library folder --> delete library folder
//
//   What is the difference between #include <filename> and #include “filename”?
//     See https://stackoverflow.com/questions/21593/what-is-the-difference-between-include-filename-and-include-filename
//     #include <file.h> will be searched for in both the sketch and core libraries, with sketch used first if found.
//     #include "file.h" is "normally used to include PROGRAMMER-DEFINED header files."
//
//   Some of these libraries may, in turn, #include other libraries, e.g., Ethernet.h
//
//   These "core" libraries are (apparently) included by DEFAULT, e.g., <math.h>, <stdlib.h>, <string.h>
//
//   DO NOT Update the Time library using Sketch --> Manage Libraries!
//     Doing creates a "Time" folder (instead of "Time-master") which then generates compile errors involving DS3231RTC.
//
//   ***ModbusMaster.h and ModbusMasterTCP HAVE BEEN MODIFIED (same constant in both)***
//    static const uint16_t ku16MBResponseTimeout          = 2000; ///< Modbus timeout [milliseconds]
//    -->
//    static const uint16_t ku16MBResponseTimeout          = 100; ///< Modbus timeout [milliseconds]
//
//   ***Ethernet.h and Ethernet.cpp HAVE BEEN MODIFIED***
//     See note below regarding SSLClient library.
//
//   ***DueFlashStorage.h HAS BEEN MODIFIED*** to allow an ABSOLUTE flash memory address (i.e., starting from 0)
//     The default is an address RELATIVE to IFLASH1_ADDR = 0xC0000. This was necessary to allow firmware updates to work properly.
//       //#define  FLASH_START  ((byte *)IFLASH1_ADDR)
//       #define  FLASH_START  ((byte *)0)
//
//   ***HttpClient.h HAS BEEN MODIFIED*** to turn OFF debug comments
//      //#define LOGGING
//



// *** #include's, #define's and GLOBAL vars ***
#include <Streaming.h>        // ~/Documents/Arduino/libraries/Streaming
#include <TimeLib.h>          // ~/Documents/Arduino/libraries/Time-master
#include <DS3231RTC.h>        // ~/Documents/Arduino/libraries/DS3231RTC-master
#include <ModbusMaster.h>     // ~/Documents/Arduino/libraries/ModbusMaster-master
#include <DueFlashStorage.h>  // ~/Documents/Arduino/libraries/DueFlashStorage-master
#include <Wire.h>             // C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\Wire\src (CORE lib)
#include <JsonParser.h>       // ~/Documents/Arduino/libraries/ArduinoJson
#include <OneWire.h>          // ~/Documents/Arduino/libraries/OneWire

// The following libs include Ethernet.h:
// For testing, in this sketch or in the following .h files...
//   #include <Ethernet.h> MUST BE CHANGED to the new Ethernet library,
//   e.g., #include <EthernetLarge.h> or #include <Ethernet_Generic.h>
// Tests so far with Ethernet_Generic and EthernetWebServer_SSL libraries generate compile errors.
#include <ModbusTCP.h>        // ~/Documents/Arduino/libraries/ModbusTCP-master


#include <HttpClient.h>       // ~/Documents/Arduino/libraries/HttpClient

#define WEBDUINO_FAVICON_DATA ""     // no favicon
#define WEBDUINO_SERIAL_DEBUGGING 2  // WebServer library debugging: 0=off, 1=requests, 2=verbose
#include <WebServer.h>        // ~/Documents/Arduino/libraries/Webduino-master

#include <SSLClient.h>        // ~/Documents/Arduino/libraries/SSLclient

// To implement SSLClient EthernetHTTPS example:
//   1. MANUALLY PATCH Ethernet.h (v2.0.1) for compatibility with SSLclient! - see https://github.com/OPEnSLab-OSU/SSLClient
//   2. Sketch --> Include Library --> Manage Libraries --> Add SSLClient --> #include <SSLClient.h>
//   3. Sketch --> Add File --> ~\Documents\Arduino\libraries\SSLClient\examples\EthernetHTTPS\trust_anchors.h
//   4. Had to add inline max() and min() functions to stepper.h - see stepper.h for details.
//      Why? Found SSLClientParameters.h has "#undef min" and "#undef max" lines! Are they necessary?
//   5. See example code in ~\Documents\Arduino\libraries\SSLClient\examples\EthernetHTTPS
//   6. MANUALLY PATCH Ethernet.cpp - see https://github.com/OPEnSLab-OSU/SSLClient#implementation-gotchas
//      Scroll down to "Known Issues" --> issue #1 --> "More information here" -->
//      https://github.com/OPEnSLab-OSU/SSLClient/issues/13#issuecomment-643855923
//      contains a fix to void EthernetClient::flush(), adding a timeout.
//   7. MANUALLY PATCH SSLClient.cpp to generate pseudo-random 8-bit numbers using random() rather than analogRead()'s. 
//      This FIXES a problem with code HANGING at this step!
//        //rng_seeds[i] = static_cast<uint8_t>(analogRead(m_analog_pin));  // this periodically FAILS and HANGS the code!
//        rng_seeds[i] = static_cast<uint8_t>(random(256));  // generate random 8-bit numbers

// Following SdFat lib BackwardCompatibility example: 
// Also, change initializeSDCard() arg in sdcard.ino
#include <SPI.h>     // SPI bus library. SD and SdFat both require <SPI.h>
#include <SdFat.h>   // replacement library for SD.h
SdFat SD;            // replace SD functions with SdFat functions
#define SD_CS_PIN 4  // SD Chip Select pin: pin 4 is the SD. Used by SdFat library!
                     // If this define is commented out, SD initialization fails, but Ethernet then succeeds including ntpudp

// THIS IS OBSOLETE but saved because there are legacy code blocks that used it... just in case
//#define OLD_TIMER  // leave this UNDEFINED!
#ifdef OLD_TIMER
#include <DueTimer.h>
#endif

//#define NOTIMER                                              // turn off main timer
//#define USE_TEST_VALS                                        // used in various modules for testing
//#define SHOW_MOTOR_STATUS                                    // used in stepper.h to monitor stepper motor

#define MSGLVL 2                                             // ***threshold for debug printing*** - see utils.ino
#define FAST_AD                                              // used below
#define I2C_ADDRESS 0x50                                     // used in utils.ino

#define PARMFILENAME "parms1.txt"                            // SD parm file name
#define BOOTLOADER_PATH "/WWE/bin/firmware/updatefw.bin"     // path including file name, leading slash required
#define SD_FILENAME "updatefw.bin"                           // bootloader binary file name only
#define SAMPLE_RATE_PER_SEC 10000                            // max ADC rate is 10000 Hz
#define SAMPLE_PERIOD_MICROS 1000000 / SAMPLE_RATE_PER_SEC   // 10^6 usec/sec * (1/10000) sec/sample = 100 usec/sample
#define DOMODBUS                                             // used below to initialize Modbus
#define MPH2MS 0.44704                                       // mph --> m/s
#define MS2MPH 2.236936                                      // m/s --> mph
#define FURLCTL_PER_SEC 10                                   // rate that furlctl() is called in furlctl.ino
#define FURLCTL1_PER_SEC 10                                  // rate that furlctl1() is called in furlctl.ino

// Choose ONLY ONE motor type - linear actuator (legacy W5 turbine) or stepper (W2 turbine):
//#define ENABLE_LINEAR_ACTUATOR  // if defined, we use furlctl() in adc.ino
#define ENABLE_STEPPER          // if defined, we use furlctl1() in adc.ino, also used in pindefs.h

// These #include'd sketch files need to be placed here and in the order listed.
//   Why? Because they reference some of the preceding #define's and code in preceding .h files.
//   Also, because they are ALL sketch tabs!
#include "parms.h"
#include "parmdefs.h"  // references MPH2MS and code in parms.h
#include "pindefs.h"   // references ENABLE_STEPPER
#include "modbus.h"
#include "temperature.h"

#ifdef ENABLE_STEPPER
#include "stepper.h"  // local sketch file
// Instantiate a stepper motor controller:
StepperMotor motor(TC2,
              2,
              MOTOR_STEP_PIN,
              MOTOR_DIR_PIN,
              MOTOR_ENBL_PIN,
              ACCELERATION_INIT,
              MIN_VELOCITY_INIT,
              MAX_VELOCITY_INIT);
#endif

#define PREFIX ""                        // used here and in webserver.ino
WebServer webserver(PREFIX, 80);         // create a webserver object
DS18B20 cardtemp(ONE_WIRE_TEMP_PIN);     // create a controller logic board temperature object - see temperature.h
DS3231RTC realtime_clock = DS3231RTC();  // create a real time clock object

boolean debounced_rs_state = HIGH;       // see debounceRS() in adc.ino
int dump_load_duty_cycle = 0;            // used in adc.ino and furlctl.ino
boolean do_post = false;                 // flag set by readADCs in adc.ino

unsigned long rtc_time = 0;              // used in loop() and web.ino
unsigned long myunixtime;                // ***PROGRAM TIME***

int windspeed_ms = 0;                    // wind speed (m/s, 1024x actual)
int last_windspeed_ms = 0;               // saved wind speed (m/s, 1024x actual)
int Ta = 0;                              // anemometer temp (degF, 1024x actual)
int Tctl = 0;                            // controller logic board temperature
int a7_val = 0;                          // VUB72 rectifier thermistor channel - see adc.ino, utils.ino
int rectifier_temp_int;                  // VUB72 rectifer temperature (1024x actual)

boolean disable_adc = false;             // used to shut down machine prior to firmware upgrade
boolean ledOn = false;                   // used in myHandler() and adc.ino

int furl_reason_saved = 0;               // saved furl_reason (used for saving a non-zero furl condition)
int sc_reason_saved = 0;                 // saved sc_reason (used for saving a non-zero SC condition)
int quiet_time = 0;                      // timer for tail exercise
boolean weather_furl = false;            // weather furl flag - see webclient.ino, furlctl.ino

int shutdown_state = 1; // 0 = normal operation
                        // 1 = 'soft' shutdown (for SYSTEM STARTUP, firmware updates) 
                        // 2 = 'hard' shutdown (for EMERGENCY CONDITIONS, doesn't reset on restart)

// These buffers MUST be large enough to hold the longest anticipated data strings!
char jsonbuf[4096];  // used here and in parms.ino, web.ino, webclient.ino
byte rcvbuf[4096];   // used in webclient.ino

char mac_chars[] = "00:00:00:00:00:00\0";  // MAC address string, used in parms.ino, web.ino, webclient.ino
boolean SD_ok = false;                     // SD card flag - see wwe.ino, parms.ino, webclient.ino

const unsigned int udp_remote_port          = 58328;  // Controller data port
const unsigned int udp_remote_port_mod_fast = 58329;  // Modbus/RTU "fast" data port
const unsigned int udp_remote_port_mod_slow = 58330;  // Modbus/RTU "slow" data port
const unsigned int udp_remote_port_config   = 58331;  // Controller configuration data port
const unsigned int udp_remote_port_nuvation = 58332;  // Modbus/TCP Nuvation data port

// These vars have been used at various times to extract data from fast-running routines like 
//   furlctl1() - which is being called at 10000 Hz and so CANNOT have Serial print statements!
// Instead, print these vars anywhere in loop(). Most may be commented out and some may no longer be used!
int print_ws_arrlen;
int print_ws_arrsum;
float print_ws_avg;  // float because we want to show 2-digit precision
float print_ws_max;  // float because we want to show 2-digit precision
float print_dWSdt;   // float because we want to show 2-digit precision
int print_hold_WSgte[41] = { };
int print_WSgte20_hold;
int print_WSgte22_hold;
int print_WSgte24_hold;
int print_WSgte26_hold;
int print_WSgte28_hold;
int print_WSgte30_hold;
int print_WSgte32_hold;
int print_WSgte34_hold;
int print_WSgte36_hold;
int print_WSgte38_hold;
int print_WSgte40_hold;

int print_rpm_arrlen;
int print_rpm_arrsum;
int print_rpm_avg;
int print_predRPM;
float print_dRPMdt;  // float because we want to show 2-digit precision

int print_TP_timer;
float print_predTP;  // float because we want to show 2-digit precision

// END #include's, #define's and GLOBAL vars



// ********** WATCHDOG **********
// The watchdogSetup() function must be present for watchdog to work!
//   Set watchdog timeout (msec) > longest expected delay OF ANY KIND in loop() code.
//   Test by putting a while(1); at the end of setup() --> TESTED, WORKING
// UNCOMMENT THIS when code is tested and ready to go!
//void watchdogSetup() {
//  watchdogEnable(30000);
//}



// ********** SETUP **********
// setup() is run ONCE when the controller is powered up or restarted.
// setup() is followed by loop() which iterates forever.

void setup() {

  // Initialize Serial monitor
  Serial.begin(115200);
  while (!Serial) {}  // wait for serial monitor to start

  Serial << "\n\n\n";
  Serial << "********************\n";
  Serial << "W2 System Controller\n";
  Serial << "********************\n";
  Serial << "wwe: Start setup()...\n";

  // Initialize Arduino pin states - see pindefs.h
  initPins();

  // Initialize I2C interface - see Wire library
  //   This is used for reading the Arduino MAC (from EEPROM) and for accessing the real-time clock.
  Wire.begin();

  // Tweak the Mode register for the ADC:
  // 1. Reduce "settling time", bits 21,20 needed if changing gain, offset or differential settings 
  // between channels, to minimum; we don't do these things.
  // 2. Drop the "startup" time way down. This is needed if the unit is disabled or coming out of 
  // sleep mode. We leave it enabled all of the time, so don't need the setup time.
  // 3. Change the prescale to make it clock at 21 MHz instead of 14 MHZ.
  // This should cut ADC time down from around 40 usec to about a tenth that.
  // See the SAM3X manual for description of this register.
#ifdef FAST_AD
  unsigned long reg;
  reg = REG_ADC_MR;
  //Serial.print("wwe: REG_ADC_MR = ");  Serial.println(reg, HEX);
  REG_ADC_MR = (REG_ADC_MR & 0xFFC000FF) | 0x00020100;
  reg = REG_ADC_MR;
  //Serial.print("wwe: REG_ADC_MR = ");  Serial.println(reg, HEX);
#endif
  delay(100);                 // Is this necessary???
  analogWriteResolution(12);  // set ADC/DAC resolution to 12 bits
  analogReadResolution(12);
  // This reduces the conversion time for the ADC - see http://www.djerickson.com/arduino/
  //   But, it doesn't seem to make any difference. Each analogRead takes about 6 usec (cw, 2015.9.18)
  //   Chuck's comment is probably why the next line is commented-out (aw)
  //REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;
  initADCOffsets();  // see adc.ino

  // Initialize shorting contactor - see furlctl.ino
  // We startup with the alternator shorted!
  initSC();
  
  // Initialize Etesian anemometer - see wind.ino
  initWind();

  // Initialize Ethernet and timekeeping - see web.ino
  //   Problems initializing Ethernet + SD went away after replacing the original Ethernet shield with the Ethernet 2 shield.
  // SETUP NOTE: It may be possible to continue WITHOUT Ethernet, but further testing is required before we allow that.
  while ( !initWeb() ) {
    Serial << "wwe: ETHERNET DOWN!\n";
    delay(15000);
    Serial << "wwe: Retrying Ethernet...\n";
  }

  // Initialize SD card - see sdcard.ino
  //   With the original Ethernet shield, the SD card would NOT initialize following a code update.
  //   A workaround was implemented with this while() loop, where ejecting then reinserting the SD card works.
  //   This problem does NOT occur with the Ethernet 2 shield.
  while ( !(SD_ok = initializeSDCard()) ) {
    Serial << "wwe: Eject and reinsert SD card.\n";
    delay(15000);
  }
  // If the SD parm file doesn't exist, create it and write current parms to it - see parms.ino
  if ( SD_ok ) if ( !readSD2Parms(PARMFILENAME) ) writeParms2SD(PARMFILENAME);

  // Initialize Modbus/RTU and Modbus/TCP - see modbus.ino
#ifdef DOMODBUS
  initModbus();
#endif

  // Initialize controller board temperature sensor.
  cardtemp.init(34);

  // Initialize Shutdown State.
  // Unless we're in SS=2, we initialize into SS=1.
  // This forces the system operator to deal with SS=2 situations, but it also means that 
  // setting SS=0 is required to start the turbine. And doing THAT requires that the controller 
  // web server (Ethernet!) be functional to provide a Controller Operating Parameters web page.
  if ( parm_shutdown_state.intVal() != 2 ) {
    parm_shutdown_state.setParmVal("1");
    shutdown_state = 1;
  }
  
  // Increment controller Restart Count parm.
  int num_resets = 1 + parm_num_resets.intVal();
  parm_num_resets.setParmVal(num_resets);
  Serial << "wwe: Controller restarts = " << num_resets << "\n";

  // Save updated parms to SD.
  // During setup(), we've possibly changed parm_shutdown_state and DEFINITELY changed parm_num_resets. 
  if ( SD_ok ) writeParms2SD(PARMFILENAME);
  printParms();  // see parms.ino

  // Start the MAIN TIMER-driven process which calls readADCs() in adc.ino at SAMPLE_RATE_PER_SEC = 10000 Hz (100 usec/sample)
  // Among other tasks, readADCs() runs the stepper motor and manages the dump load.
#ifdef OLD_TIMER
  //Timer.getAvailable().attachInterrupt(myHandler).start(500000);               // myHandler() toggles an LED - see utils.ino
  Timer.getAvailable().attachInterrupt(readADCs).start(SAMPLE_PERIOD_MICROS);  // = 1000000/SAMPLE_RATE_PER_SEC = 100 usec/sample
  Serial << "wwe: ***OLD TIMER started***\n";
#else
  //pmc_set_writeprotect(false);     // disable write protection for pmc registers
  //pmc_enable_periph_clk(ID_TC0);   // enable peripheral clock TC7 so we can configure timer.
  //pmc_enable_periph_clk(ID_TC1);
  //pmc_enable_periph_clk(ID_PIOD);
  
  // There are three timer units on the chip, TC0, TC1, TC2. Each has three timers.
  //   These 9 timers are also sometimes referred to as TC0 to TC8.
  //   TC2, 1 (aka TC7) drives the state machine (constant rate) - Per usage below, should this be TC0, 0 (aka TC0)?
  //   TC2, 2 (aka TC8) drives the stepper motor (variable rate)
  // Set up a timer interrupt for SAMPLE_RATE_PER_SEC to drive the state machine.
  startTimer(ID_TC0, TC0, 0, TC0_IRQn, 1, SAMPLE_RATE_PER_SEC);
  Serial << "wwe: ***MAIN TIMER started***\n";
#endif

  // Initialize a timer-driven interrupt for motor stepping.
  // Timer interrupts run at 2x the rate of the stepper motor clock, so we initialize to 2x the initial stepper clock frequency.
#ifdef ENABLE_STEPPER
  startTimer(ID_TC8, TC2, 2, TC8_IRQn, 0, MIN_VELOCITY_INIT * 2);
#endif


  // check weather data as we startup, then at intervals in loop()
  getWeatherData();  // in weather.ino --> calls getNWSAPIData and getOpenMeteoData in webclient.ino
  Serial << "wwe: weather_furl = " << weather_furl << "\n";

  
  //while(1);  // DEAD END. Run setup() code above this only.
  
  Serial << "wwe: End setup().\n";
}  // END setup()
// ********** END SETUP **********




// ********** LOOP **********
// loop() runs at some free-running rate and can be used for whatever processing is NOT in if(do_post){}, 
//   i.e., the do_post flag is set precisely at once per second intervals from readADCs() in adc.ino.
//   So, if(do_post){...} code will be executed once per second.
// But...
//   If the if(do_post){...} code requires > 1sec to execute, then we'll only see 1 loop() iteration per do_post()!
//   If the if(do_post){...} code requires < 1sec to execute, we can see several 1000 loop() iterations!
//
// However, because the timer sets a flag, we CANNOT assume that we'll enter if (do_post) {} at exactly the same 
//   moment each second! If that is important, then perhaps we should consider calling if(do_post) from a timer interrupt. 
//   Or, at least carve out that code which should be executed precisely on the second... perhaps the Modbus read code?

void loop() {
  // First things first... reset watchdog every loop() iteration!
  //   ***watchdogSetup() must be UNCOMMENTED ABOVE for watchdog to work***
  watchdogReset();

  static int first_loop = true;           // first iteration of loop() flag
  static unsigned int post_counter = 0;   // post iteration counter, 2^32-1 = 4,294,967,295 seconds ~ 136.19 years!
  static auto do_loop_time = millis();    // cumulative time for loop() iterations between POST's
  static int loop_counter = 0;            // # loop() iterations between POST's
  static int led_counter = 0;             // loop() LED counter
  static int led_state = HIGH;            // loop() LED on/off state

  
  if ( first_loop ) {
    first_loop = false;
    Serial << "wwe: Start loop()...\n";
  }

// This is left over from early testing...
#ifdef USE_TEST_VALS
  getTestInputs();  // see web.ino
#endif

  // Toggle an LED on/off with loop() iterations.
  //   ~30000 loop() cycles per if(do_post), and therefore, per second are typical.
  if ( led_counter++ >= 5000 ) {
    led_state == HIGH ? led_state = LOW : led_state = HIGH;
    led_counter = 0;
    digitalWriteDirect(MAIN_LOOP_LED_PIN, led_state);
  }



  // ***TIMER LOOP***
  //   ***ONLY PUT CODE HERE THAT MUST BE EXECUTED AT 1-SECOND INTERVALS***
  //   The do_post flag is set by readADCs() in adc.ino at EXACTLY 1-sec intervals, so...
  //     everything in if(do_post){} will execute at 1-sec intervals UNLESS:
  //     1. code in loop() delays entry into if(do_post), e.g., a weather API request, or
  //     2. code inside if(do_post) cumulatively takes longer than 1-sec.
  //   This is why, with timers and Serial statements, we monitor how long various processes 
  //     take in loop() and if(do_post) --> We want to limit loop() iteration time to 1-sec MAX.
  //     Why? Because the intent is to gather system data with 1-sec resolution!
  //     When do_post == true, if(do_post){} will use up MOST of that 1-sec, reading, writing and posting data.
  //     When do_post == false, if(do_post){} is skipped over and loop() free-runs around it, accumulating
  //     1 to several 1000's of iterations in the time remaining before do_post is set true again.
  if ( do_post ) {
    auto do_post_time = millis(); 

    Serial << "wwe: ++++++++++ Starting POST #" << ++post_counter << " ++++++++++\n";

    // ***CHECK SYSTEM STATE***
    // Write the current PARM value of shutdown_state to the GLOBAL working var
    shutdown_state = parm_shutdown_state.intVal();  
    if (shutdown_state == 0) Serial << "wwe: NORMAL OPERATION\n";
    if (shutdown_state == 1) Serial << "wwe: SHUTDOWN STATE = 1\n";
    if (shutdown_state == 2) Serial << "wwe: SHUTDOWN STATE = 2\n";

    // ***CHECK TIME***
    // Update myunixtime every hour, on-the-hour from the RTC
    // ***myunixtime is incremented EVERY SECOND by readADCs() - see adc.ino***
    if ( rtc_time = realtime_clock.get() ) {
      if ( rtc_time % 3600 == 0 ) myunixtime = rtc_time;
    }
    // Display UTC and LOCAL time for this post
    // ***myunixtime is incremented EVERY SECOND by readADCs() - see adc.ino***
    time_t t = myunixtime;
    char theyear[] = "yyyy", themonth[] = "mm", theday[] = "dd", thehour[] = "hh", theminute[] = "mm", thesecond[] = "ss";
    sprintf(theyear, "%d", year(t)); sprintf(themonth, "%02d", month(t)); sprintf(theday, "%02d", day(t));
    sprintf(thehour, "%02d", hour(t)); sprintf(theminute, "%02d", minute(t)); sprintf(thesecond, "%02d", second(t));
    Serial << "wwe: RTC time   = " << theyear << "-" << themonth << "-" << theday << " " << thehour << ":" << theminute << ":" << thesecond << "\n"; 
    t += parm_TZ_offset.intVal()*3600;  // add timezone offset
    sprintf(theyear, "%d", year(t)); sprintf(themonth, "%02d", month(t)); sprintf(theday, "%02d", day(t));
    sprintf(thehour, "%02d", hour(t)); sprintf(theminute, "%02d", minute(t)); sprintf(thesecond, "%02d", second(t));
    Serial << "wwe: LOCAL time = " << theyear << "-" << themonth << "-" << theday << " " << thehour << ":" << theminute << ":" << thesecond << "\n"; 

    // ***HANDLE WEBSERVER CONNECTIONS*** --> ***REQUIRES Ethernet***
    // A simple webserver generates the Controller Operating Parameters web page at <controllerIP>/parms.html 
    //   among other requests, all of which .processConnection() handles - see webserver.ino.
    // This WAS outside of if(do_post) and required a few *msec* each loop() iteration, which adds up, so we put it here.
    if ( ethernetOK() ) {
      Serial << "wwe: CHECKING WEBSERVER...\n";
      webserver.processConnection();
    }

    // ***RESET MORNINGSTAR COUNTERS***
    //   These resettable Ah and kWh counters are reset 00:00 UTC time --> ***does NOT require Ethernet***
    //   We do this here because putting it outside of if(do_post) in free-running loop() could result in multiple executions.
    //   ***The 5 msec delays appear to be necessary.*** One could experiment with shorter delays.
    if ( (myunixtime % 86400) == 0 ) {
      Serial << "wwe: RESETTING MORNINGSTAR Ah COUNTERS...\n";
      int result = mppt600.writeSingleCoil(0x0010, 1); delay(5);  // WIND
      Serial << "wwe: mppt600 Ah reset = " << result << "\n";
      result = mppt30.writeSingleCoil(0x0010, 1); delay(5);       // PV1
      Serial << "wwe: mppt30 Ah reset = " << result << "\n";
      result = mppt60.writeSingleCoil(0x0010, 1); delay(5);       // PV2
      Serial << "wwe: mppt60 Ah reset = " << result << "\n";
      result = div60.writeSingleCoil(0x0010, 1); delay(5);        // DIV1
      Serial << "wwe: div60 Ah reset = " << result << "\n";
      result = div2.writeSingleCoil(0x0010, 1); delay(5);         // DIV2
      Serial << "wwe: div2 Ah reset = " << result << "\n";

      Serial << "RESETTING MORNINGSTAR kWh COUNTERS...\n";
      result = mppt600.writeSingleCoil(0x0012, 1); delay(5);      // WIND
      Serial << "wwe: mppt600 kWh reset = " << result << "\n";
      result = mppt30.writeSingleCoil(0x0012, 1); delay(5);       // PV1
      Serial << "wwe: mppt30 kWh reset = " << result << "\n";
      result = mppt60.writeSingleCoil(0x0012, 1); delay(5);       // PV2
      Serial << "wwe: mppt60 kWh reset = " << result << "\n";
    }

    // ***GET WEATHER DATA*** --> ***REQUIRES Ethernet***
    //   To avoid Ethernet adapter use conflicts with other processes, we do the API call(s) at
    //     some arbitrary # seconds after the hour mark, also not falling on-the-minute (like config requests).
    if ( (myunixtime % 600) == 6 ) {
      if ( ethernetOK() ) {
        auto starttime = millis();
        Serial << "wwe: getWeatherData() called.\n";
        getWeatherData();  // in weather.ino --> calls weather API(s) in webclient.ino
        Serial << "wwe: weather_furl = " << weather_furl << "\n";
        Serial << "wwe: getWeatherData read time = " << (millis() - starttime) << " msec\n";
      }
    }

    
    
    // ***READ DATA***
    Serial << "wwe: READING DATA...\n";
    
    // READ Modbus/RTU 'fast' data into cache --> ***does NOT require Ethernet***
    //   Total read time must be < 1 sec! To minimize read time, we do "long reads" to fill 'cache' buffers. 
    //   The <device>_cache objects are defined in modbus.h and can be accessed much faster than serial single register reads.
    //   See also code in modbus.ino that sets # regs to be read into the cache, using setNumRegs().
    //   The delay(5) after each readReg() is REQUIRED to avoid a "Response Timeout" error - see modbus.h
    //     One could experiment with shorter delays.
    auto modbustime = millis(); mppt600_cache.readReg(false); delay(5);  // WIND
    Serial << "wwe: MPPT-600V Modbus/RTU read time = " << (millis() - modbustime) << " msec\n";

    modbustime = millis(); mppt30_cache.readReg(false); delay(5);        // PV1
    Serial << "wwe: MPPT-30 Modbus/RTU read time = " << (millis() - modbustime) << " msec\n";

    modbustime = millis(); mppt60_cache.readReg(false); delay(5);        // PV2
    Serial << "wwe: MPPT-60 Modbus/RTU read time = " << (millis() - modbustime) << " msec\n";

    modbustime = millis(); div60_cache.readReg(false); delay(5);         // DIV1
    Serial << "wwe: TS-60(1) Modbus/RTU read time = " << (millis() - modbustime) << " msec\n";

    modbustime = millis(); div2_cache.readReg(false); delay(5);          // DIV2
    Serial << "wwe: TS-60(2) Modbus/RTU read time = " << (millis() - modbustime) << " msec\n";

    /*
    // Read Modbus/RTU 'slow' data. 
    //   Will this take too long?
    if ( (myunixtime % 3600) == 0 ) {  // read Modbus/RTU 'slow' data every hour, on-the-hour
      mod_starttime = millis();
      for (int i = 0; i < NUM_MOD_SLOW_CHANNELS; i++) {
        mod_slow_regs[i]->readReg(false);  // readReg(false) means read directly from the device
      }
      Serial << "wwe: Modbus/RTU 'slow' read time = " << (millis() - mod_starttime) << " msec\n";
    }
    */

    // READ Modbus data from cache --> ***does NOT require Ethernet***
    for (int i = 0; i < NUM_MOD_FAST_CHANNELS; i++) {
      //
      // This if() is a HACK to skip over a few 'slow' regs that we've put (for now) in the mod_fast_regs[] array for convenience.
      // Specifically, the HVD and HVR regs of the PV controllers. We have to do this because they're NOT in the long read (cache) buffers!
      char* fast_chan_ptr = mod_fast_regs[i]->getChanName();
      if ( !strcmp(fast_chan_ptr, mppt60_EV_hvd.getChanName()) ||  // strcmp() returns 0 if strings match, so !strcmp() will ==1 if there's a match
           !strcmp(fast_chan_ptr, mppt60_EV_hvr.getChanName()) ||
           !strcmp(fast_chan_ptr, mppt30_EV_hvd.getChanName()) ||
           !strcmp(fast_chan_ptr, mppt30_EV_hvr.getChanName()) ) {
        //Serial << "wwe: STRING MATCH FOUND\n";  // for debug
        continue;  // continue loop at the next value of i, skipping over the line below which reads from cache
      }
      // mod_fast_regs[] is an array of POINTERS to regs that we're reading every second - see modbus.h
      mod_fast_regs[i]->readReg(true);  // true --> read cached regs found at getResponseBuffer(response_buffer_offset)
                                        // associated with a particular mod_dev_ptr or mod_dev_ptr_tcp (see modbus.h).
      //Serial << getModchannelName(1, i) << ": " << getModchannelValue(1, i) << "\n";
    }

    // READ Etesian anemometer data --> ***does NOT require Ethernet***
    // If Serial2.print("T\r\n"); is put into processSerialWind(), if (do_post) slows down dramatically! (about 4 sec/iteration). WHY???
    //   Regardless, putting the Serial2 data request here fixes the problem.
    //   TEST: If we move the data request and processSerialWind() out of if(do_post), but within loop(), we seem to get
    //         finer-grained data (artifactual?), but we also get watchdog resets varying from seconds to minutes! WHY???
    auto wind_starttime = micros();
    Serial2.print("T\r\n");             // request a single line of anemometer data
    processSerialWind();                // process new anemometer data (if any) - see wind.ino
    Serial.print("wwe: Etesian wind data read time = ");
    Serial.print( ((float)(micros() - wind_starttime)/1000.), 3);
    Serial.println(" msec");

    // READ controller board temperature --> ***does NOT require Ethernet***
    Tctl = cardtemp.readTemp()*1024;    // controller board temp
    cardtemp.convert();                 // convert() is AFTER readTemp() because a 1-sec-ish delay is required BEFORE readTemp()
                                        // - see temperature.h. Because if(do_post){...} runs at 1 Hz, it provides this delay.
    //Serial << "wwe: Tc = " << Tctl << " degC (1024x actual)\n";

    // READ rectifier board temperature --> ***does NOT require Ethernet***
    rectifier_temp_int = (int)(1024.0 * calcThermistorTemp(2200, 3560, 978, 3.3));
    //Serial << "wwe: Tr = " << rectifier_temp_int << " degC (1024x actual)\n";


    // READ Nuvation data --> ***REQUIRES Ethernet***
    if ( ethernetOK() ) {
      auto tcp_starttime = millis();
      for (int i = 0; i < NUM_MOD_NUV_CHANNELS; i++) {
        mod_nuv_regs[i]->readReg(false);  // false --> read directly from the device (Nuvation is quite fast)
        //Serial << getModchannelName(3, i) << ": " << getModchannelValue(3, i) << "\n";
      }
      // If this times out, the total read time will be ~24 sec = 8 * 3000 ms.
      // 3000 ms is hard-coded in ModbusTCP.cpp. Change this?
      Serial << "wwe: Nuvation Modbus/TCP read time = " << (millis() - tcp_starttime) << " msec\n";
    }


    // ***WRITE DATA TO SD***
    Serial << "wwe: WRITING DATA...\n";
    
    // WRITE controller data to SD. ***does NOT require Ethernet***
    char control_fname[] = "yyyymmdd.ctl";                              // create 8.3 format SD filename based on LOCAL time
    sprintf( control_fname, "%s%s%s.ctl", theyear, themonth, theday );  // make controller data filename
    auto sd_starttime = millis();
    writeDataToSD(control_fname, 0);  // 0 = controller data, see sdcard.ino
    Serial << "wwe: SD write time (Controller data) = " << millis() - sd_starttime << " msec\n";

    // COMMENTED OUT these SD writes because they may cause if (do_post) to exceed 1 second (2022-02-25)
    //   We'll monitor if(do_post) timing to see if we can allow one or more of them.
    //   The controller data are most important, so having it saved on SD is useful if Ethernet fails.
    //   When Ethernet is up, ALL device data are being sent to and saved on the Data Server anyway.
    /*
    // Write Modbus 'fast' data to SD.  ***does NOT require Ethernet***
    char modfast_fname[] = "yyyymmdd.fst";
    sprintf( modfast_fname, "%s%s%s.fst", theyear, themonth, theday );
    sd_starttime = millis();
    writeDataToSD(modfast_fname, 1);                                     // 1 = Modbus fast data, see sdcard.ino
    Serial << "wwe: SD write time (Modbus fast data) = " << millis() - sd_starttime << " msec\n";

    // WRITE Modbus 'fast' data to SD.  ***does NOT require Ethernet***
    if ( (myunixtime % 3600)==0 ) {
      sd_starttime = millis();
      char modslow_fname[] = "yyyymmdd.slw";
      sprintf( modslow_fname, "%s%s%s.slw", theyear, themonth, theday );
      writeDataToSD(modslow_fname, 2);                                   // 2 = Modbus slow data, see sdcard.ino
      Serial << "wwe: SD write time (Modbus slow data) = " << millis() - sd_starttime << " msec\n";
    }

    // WRITE Nuvation data to SD. ***does NOT require Ethernet***
    sd_starttime = millis();
    char nuvation_fname[] = "yyyymmdd.nuv";
    sprintf( nuvation_fname, "%s%s%s.nuv", theyear, themonth, theday );
    writeDataToSD(nuvation_fname, 3);                                    // 3 = Nuvation data, see sdcard.ino
    Serial << "wwe: SD write time (Nuvation data) = " << millis() - sd_starttime << " msec\n";
    */


    // ***POST DATA*** --> ***requires Ethernet***
    Serial << "wwe: POSTING DATA...\n";
    
    if ( ethernetOK() ) {
        
      //sendUDPWorkaround();  // Is this necessary with Ethernet Shield 2? - see web.ino

      // Send UDP data to the Data Server.
      //   Program flow: sendDataUDP() --> printPOSTBody() --> printOrCount(). See web.ino and webclient.ino
      sendDataUDP(parm_udp_ip.parmVal(), udp_remote_port, 0);             // port = 58328, 0 = Controller data
 
      sendDataUDP(parm_udp_ip.parmVal(), udp_remote_port_mod_fast, 1);    // port = 58329, 1 = Modbus/RTU 'fast' data

      if ( (myunixtime % 3600) == 0 ) {                                   // Every hour, on-the-hour...
        sendDataUDP(parm_udp_ip.parmVal(), udp_remote_port_mod_slow, 2);  // port = 58330, 2 = Modbus/RTU 'slow' data
      }

      sendDataUDP(parm_udp_ip.parmVal(), udp_remote_port_nuvation, 3);    // port = 58332, 3 = Modbus/TCP nuvation data

      // Send a CONFIG REQUEST to the Data Server.
      //   A python script, cfgudp.py, on the Data Server compares the controller config (aka controller operating parameters) 
      //   to the server's version of the same. The script EXCLUDES some parms from comparison, e.g., Shutdown State and HVDL Active.
      if ( (myunixtime % (60*parm_cfg_minutes.intVal())) == 0 ) {  // every parm_cfg_minutes...

        Serial << "wwe: Checking for update to controller operating parameters...\n";
        writeParms2Buff(true);  // write current controller parms to jsonbuf; true --> include controller MAC
                                // The MAC is needed to access the controller's unique MAC-coded config file on the Data Server
        //printJSONBuf();  // print contents of jsonbuf, see parms.ino
          
        int config_rc = sendConfigUDP( parm_udp_ip.parmVal() );  // send config request - see web.ino

        // If return code bit 0 == 1 ... update parms
        if ( config_rc & 1 ) {
          Serial << "wwe: Updated parms found on Data Server. Writing new parms to SD.\n";  
          if ( SD_ok ) {                  //   if SD card is OK...
            writeParms2SD(PARMFILENAME);  //     write updated parms to SD
          } else {
            Serial << "wwe: SD is not OK! Unable to write updated parms to SD.\n";
          }
        }
        // If return code bit 1 == 1 ... ***UPDATE FIRMWARE***
        //   PROGRAM FLOW:
        //     1. doFWUpdate() loads updatefw.bin (from Update Server) into flash and reboots --> 
        //     2. updatefw.bin runs, loads new firmware (from Update Server) into flash and reboots --> 
        //     3. new firmware runs
        if ( config_rc & 2 ) {
          if ( SD_ok ) {
            Serial << "\n\n\n";
            Serial << "wwe: ---------------\n";
            Serial << "wwe: FIRMWARE UPDATE\n";
            Serial << "wwe: ---------------\n";
            Serial << "wwe: Setting Shutdown State = 1...\n";
            parm_shutdown_state.setParmVal("1");
            shutdown_state = 1;
            Serial << "wwe: Waiting 30 seconds for shutdown to complete...\n";
            delay(30000);
            Serial << "wwe: Disabling controller ADC...\n";  // ???necessary???
            disable_adc = true;
            doFWUpdate();
            // ***DEAD END***
          } else {
            Serial << "wwe: SD is not OK! Unable to update firmware.\n";
            Serial << "wwe: SD is *required* for remote firmware update.\n";
            Serial << "wwe: Manually upload and run SdInfo to check SD.\n";
          }
        }  // END do firmware update
      }  // END send config request
    } // END if ( ethernetOK() ) { <post data> }
    
    Serial << "wwe: LOOP time = " << (millis() - do_loop_time) << " msec for " << loop_counter << " loop() iterations\n";
    do_loop_time = millis();
    loop_counter = 0;
    Serial << "wwe: POST time = " << (millis() - do_post_time) << " msec\n";
    Serial << "wwe: ++++++++++ POST #" << post_counter << " complete ++++++++++\n";

    // We put a short delay here to allow the Ethernet adapter to 'settle' after the UDP sends.
    // Because... the first task in the next if(do_post){} will be to check Ethernet with ethernetOK()
    delay(10);
    
    // Setting do_post = false MUST BE DONE HERE because readADCs sets do_post = true BY TIMER *regardless* of where we 
    //   happen to be in the loop(){ ... if(do_post){...} } cycle! For example, if we set do_post = false at the start of 
    //   if(do_post), readADCs may set do_post = true DURING if(do_post), triggering another if(do_post) cycle IMMEDIATELY 
    //   after the current if(do_post){...} completes! 
    do_post = false;
  }  // END if (doPost)

  loop_counter++;
}  // END loop()
// ********** END MAIN LOOP **********



// OTHER FUNCTIONS:

// Black magic
// Parameters table:
// ID_TC0, TC0, 0, TC0_IRQn, priority, frequency  =>  TC0_Handler()
// ID_TC1, TC0, 1, TC1_IRQn, priority, frequency  =>  TC1_Handler()
// ID_TC2, TC0, 2, TC2_IRQn, priority, frequency  =>  TC2_Handler()
// ID_TC3, TC1, 0, TC3_IRQn, priority, frequency  =>  TC3_Handler()
// ID_TC4, TC1, 1, TC4_IRQn, priority, frequency  =>  TC4_Handler()
// ID_TC5, TC1, 2, TC5_IRQn, priority, frequency  =>  TC5_Handler()
// ID_TC6, TC2, 0, TC6_IRQn, priority, frequency  =>  TC6_Handler()
// ID_TC7, TC2, 1, TC7_IRQn, priority, frequency  =>  TC7_Handler()
// ID_TC8, TC2, 2, TC8_IRQn, priority, frequency  =>  TC8_Handler()

void startTimer(uint32_t id, Tc *tc, uint32_t channel, IRQn_Type irq, 
                uint32_t priority, uint32_t frequency) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(id);
  // Set up to generate a free running wave. Source clock is timer_clock1, meaning MCK/2 (42E6),
  // resets when RC reaches the specified value.
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  // Calculate value of RC for the specified frequency, based on timer counter being incremented at the 42E6/sec rate.
  // For example, with a 10,000Hz frequency, RC is 4200. 4200 times the period of a 42MHz clock gives a 100 uSec period.
  uint32_t rc = 42000000 / frequency;
  // RA specifies when the waveform goes low.
  TC_SetRA(tc, channel, rc/2); //50% high, 50% low
  // RC specifies the period, in number of clocks, as selected in the config, this being MCK/2.
  TC_SetRC(tc, channel, rc);
  // Start the timer at rate determined by RC and the general setup.
  TC_Start(tc, channel);
  // set up the timer to cause an interrupt once per cycle.
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_SetPriority(irq, priority);
  // Enable the interrupt
  NVIC_EnableIRQ(irq);
}



// This function downloads and executes updatefw.bin.
// updatefw.bin is a BOOTLOADER program that is compiled completely separately from this code and 
//   is linked to run from flash ***Bank 1*** at address 0xC0000 in Arduino SAM DUE flash memory.
//   See https://microchipsupport.force.com/s/article/SAM3X---Switching-between-the-Flash-banks-0-and-1
//
// Once started, updatefw.bin queries the Update Server and accesses its MAC-specific config file which
//   contains a parameter "binary_filename" that specifies the name of a new firmware binary file.
//   This file is downloaded, verified, put it into flash memory ***Bank 0*** at address 0x80000 
//   (the usual location for Arduino code), and executed.
boolean doFWUpdate() {
  char cfg_addr[20];  // Update Server IP address string, e.g., "192.168.1.4"
  uint16_t cfg_port;  // Update Server port, e.g., 80 or 49152
  
  strcpy(cfg_addr, parm_cfg_ip.parmVal());  // get Update Server IP address from its parm
  cfg_port = parm_cfg_port.intVal();        // get Update Server port from its parm

  // get file at BOOTLOADER_PATH and write to SD file SD_FILENAME - see webclient.ino
  if ( getHttp2File(cfg_addr, cfg_port, BOOTLOADER_PATH, SD_FILENAME) ) {

    // NOTE: See the change to library file DueFlashStorage.h to allow an ABSOLUTE flash memory address, 
    // e.g., 0x80000 or 0xC0000. The default was an address RELATIVE to IFLASH1_ADDR = 0xC0000.
    //showNFlashBytes(40, 0xC0000);  // for DEBUG, before write to flash
    Serial << "wwe: Writing " << SD_FILENAME << " to Arduino flash at 0xC0000...\n";
    writeFile2Flash(SD_FILENAME, 0xC0000);  // write "updatefw.bin" from SD to Arduino flash at 0xC0000.
    //showNFlashBytes(40, 0xC0000);  // for DEBUG, after write to flash

    Serial << "wwe: GPNVM Security Bit (bit 0):      0 = unset, 1 = set\n";
    Serial << "wwe: GPNVM Boot Mode (bit 1):         0 = boot from ROM, 1 = boot from FLASH\n";
    Serial << "wwe: GPNVM Flash Boot Region (bit 2): 0 = FLASH0, 1 = FLASH1\n";
    Serial << "wwe: GPNVM bits = 0b" << _BIN(getGPNVMBits(EFC0)) << "\n";
    Serial << "wwe: Setting GPNVM bit 2...\n";
    __disable_irq();
    flash_set_gpnvm(2);   // set flash select bit --> on reboot, flash 0xC0000 will be mapped to 0x0, and updatefw.bin will execute from there
    __enable_irq();
    Serial << "wwe: GPNVM bits = 0b" << _BIN(getGPNVMBits(EFC0)) << "\n";
    
    Serial << "wwe: REBOOTING...\n";
    Serial.flush();

    // See https://forum.arduino.cc/t/arduino-due-software-reset/485276/5
    //   It's unclear from the discussion, but there are apparently 3 alternatives/options here.
    //   Deduced this after looking at the Atmel datasheet:
    // https://www.keil.com/dd/docs/datashts/atmel/sam3xa/doc11057.pdf - search for "software reset"
    __DSB;
    //SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);    // Option 1: reboots, but SD fails to initialize
    RSTC->RSTC_CR = RSTC_CR_KEY(0xA5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;           // Option 2: *** THIS WORKS!!! ***
    //NVIC_SystemReset();                                                             // Option 3: didn't try this

    // Other reset code that failed:
    // See https://stackoverflow.com/questions/56928363/how-do-i-close-or-reset-via-arduino-code
    // Hangs. Does not reboot!
    // What if... address 0 were changed to 0xC0000 ? (not tested)
    //void(* resetFunc) (void) = 0;  // declare reset function at address 0 
    //resetFunc();                   // call reset

    // Original reset code (from cw) - reboots, but SD fails to initialize.
    //pinMode(51, OUTPUT);
    //digitalWrite(51, LOW);
    //binary_exec((void*)0);
  }
}



#ifndef OLD_TIMER
// Interrupt service routine, called STATE_MACHINE_RATE times per second. 
// This evaluates the motor state machine.
void TC0_Handler() {
  // don't know why following is necessary, but it doesn't work without it.
  TC_GetStatus(TC0, 0);
  readADCs();
}
#endif


#ifdef ENABLE_STEPPER
// This is an interrupt service routine called by motor timer.
void TC8_Handler() {
  // Don't know why following is necessary, but it doesn't work without it!
  TC_GetStatus(TC2, 2);
  motor.handleMotorInterrupt();
}
#endif
