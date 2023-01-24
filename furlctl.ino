// ---------- furlctl.ino ----------

// This module manages TAIL MOTION, DUMP LOAD, and SHORTING CONTACTOR.
//
// IMPORTANT: 
// #define JUST ONE motor type in wwe.ino: ENABLE_LINEAR ACTUATOR or ENABLE_STEPPER
// furctl() is called in adc.ino between #ifdef ENABLE_LINEAR_ACTUATOR ... #endif
// furlctl1() is called in adc.ino between #ifdef ENABLE_STEPPER ... #endif
// -------------------------------------------------------------------
// furlctl() manages tail motion controlled by a LINEAR ACTUATOR motor.
// This function is called FURLCTL_PER_SEC times per second via timer interrupt.
// It implements a finite state machine with 2 states: UNFURL and FURL. 
// In addition, there is a countdown timer, furl_motor_on, which manages how 
// long the furl motor turns on at the beginning of the furl or unfurl state.
// There are 2 SSR's for furling and unfurling controlled by D29 and D30.
// -------------------------------------------------------------------
// furlctl1() manages tail motion controlled by a STEPPER motor.
// This function is called FURLCTL1_PER_SEC times per second via timer interrupt.
// ------------------------------------------------------------------- 


// GLOBAL VARIABLES
int sc_failsafe_reason = 0;       // return val from checkSCNowConditions()
int sc_shorted = 1;               // 0 = unshorted, 1 = shorted

int furl_reason = 0;              // bitfield encoding possible furl reasons:     VOLT, CURR, RPM, WIND, EXER, ANEM, SLIP, MORN, XWIND
int sc_reason = 0;                // bitfield encoding possible shorting reasons: VOLT, CURR, RPM, ----, EXER, ANEM, ----, MORN, XWIND
boolean engage_sc = false;        // flag for engaging shorting contactor

boolean exercise_furl = false;    // flag for tail exercise (see quiet_time)
boolean anemometer_furl = false;  // flag for frozen anemometer furl
boolean slippage_furl = false;    // flag for tail motor slippage furl
boolean tp_init_fail = false;     // flag for TP initialization failure
int morningstar_furl = 0;         // return val from checkMorningstarFaults() + checkMorningstarState()

// furlctl() vars
const int FURL = 2;                                      // Furling state machine state. FURLed.
const int UNFURL = 0;                                    // Furling state machine state. UNFURLed.
int furl_state = FURL;                                   // Initialize finite state machine to FURLed
const int FURL_RUN_TIME = 15*FURLCTL_PER_SEC;            // Motor run time when furling or unfurling, #furlctl() iterations, 15 seconds
int furl_motor_on = FURL_RUN_TIME;                       // Motor power ON countdown TIMER, #furlctl() iterations
int furl_duration = 60*FURLCTL_PER_SEC;                  // furl duration, #furlctl() iterations, 60 seconds
int unfurl_countdown = furl_duration;                    // furl duration TIMER, #furlctl() iterations
int last_furl_reason = 0;                                // previous furlctl() iteration's furl_reason, used for detecting if a new furl condiition has JUST occurred
int sc_failsafe_countdown = 30*FURLCTL_PER_SEC;          // # furlctl() iterations before shorting, 30 seconds


// RSlimit = tail position where the reed switch closes. Used in furlctl1() and checkSCNowConditions().
//   This WILL VARY with magnet strength, magnet position, and sensor position; so, RSlimit should be determined for EACH TURBINE.
//   ???Should RSlimit be a Controller Operating Parameter so that it can be tweaked on-the-fly for each turbine???
const int RSlimit = 27900;       // 81 deg = 62*2000*(81/360) = 27900 usteps
//const int RSlimit = 28589;       // 83 deg = 62*2000*(83/360) = 28589 usteps
//const int RSlimit = 29278;       // 85 deg = 62*2000*(85/360) = 29278 usteps
//const int RSlimit = 29967;       // 87 deg = 62*2000*(87/360) = 29967 usteps
//const int RSlimit = 30656;       // 89 deg = 62*2000*(89/360) = 30656 usteps

// FURLlimit is the absolute max TP and should be set > RSlimit by several deg.
// IMPORTANT: FURLlimit must be >= parm_sc_failsafe_tp because the latter sets SS=2 if exceeded.
const int FURLlimit = ((62*2000*110)/360) + 1;  // usteps, integer math --> add 1 in case parm_sc_failsafe_tp is also 99 deg.

// Create a thresholdChecker class for all kinds of threshold checking of CONTROLLER channels.
// Doing threshold checking with a class instead of just a simple compare gives us the flexibility of
// tracking selected threshold crossings or doing other cool things.
// These controller channels are defined in adc.ino:
// L1_VOLTAGE = 0;
// L2_VOLTAGE = 1;
// L3_VOLTAGE = 2;
// DC_VOLTAGE = 3;
// LINE_VOLTAGE = 4;
// L1_CURRENT = 5;
// L2_CURRENT = 6;
// L3_CURRENT = 7;
// DC_CURRENT = 8;
// L1L2_VOLTAGE = 9;
// L2L3_VOLTAGE = 10;
// L3L1_VOLTAGE = 11;
// RPM = 12;  // formerly FREQUENCY
// WINDSPEED = 13;
// DUMP_LOAD_P = 14;
// RECT_TEMP = 15;
// AMBIENT_TEMP = 16;
// CONTROLLER_TEMP = 17;
// TAIL_POSITION = 18;
// STATE = 19;

class thresholdChecker {
  public:
    thresholdChecker(char* checker_name, int channel_num, Parm* threshold_parmptr, boolean gteq, boolean record_violations): 
                     checker_name(checker_name), channel_num(channel_num), parmptr(threshold_parmptr), gteq(gteq), record_violations(record_violations),
      result(false) {}

    boolean check(boolean verbose) {
      // First, get the threshold against which we compare the measured val
      int threshold = parmptr->floatValInt();  // floatValInt() returns floatval_int = (int)(1024.0 * floatval) --> result is 1024x actual!
                                               // NOTE: floatval_int is set ONLY for FLOAT parms!
      
      // val is 1024x actual ONLY for the 9 physical analog data channels: 3 AC voltages, DC voltage, Line Voltage, 3 AC currents, and DC current.
      // This 1024x scale factor is applied by the AnalogChannel class read() method.
      // getChannelRMSInt(TAIL_POSITION) returns val as motor.currentPosition() in usteps and is NOT 1024x actual.
      int val = getChannelRMSInt(channel_num);                 

      // Special handling for TP threshold and val
      if (channel_num == TAIL_POSITION) {
        threshold = ((threshold*2000)/360)*62;  // deg --> usteps (1024x actual because of floatValInt conversion above)
        val = abs(val) << 10;                   // abs() because TP val may be (-), then multiply by 1024x to compare with threshold
      }

      if (verbose) Serial << "furlctl: checker_name = " << checker_name << ", threshold = " << threshold << ", val = " << val << "\n";

      // Compare val with threshold and set result = true or false
      gteq ? result = (val >= threshold) : result = (val <= threshold);

      if (result && record_violations) {
        // use this to save a breadcrumb?
      }
      return (result);
    }

  private:
    boolean result;
    char* checker_name;
    int channel_num;
    Parm* parmptr;
    boolean gteq;
    boolean record_violations;
};

// Define threshold names corresponding to elements of the threshold_checkers[] array (defined below).
// e.g., to check the furling voltage threshold: threshold_checkers[FURL_INIT_V]
#define FURL_INIT_V 0
#define FURL_INIT_I 1
#define FURL_INIT_RPM 2
#define FURL_INIT_WS 3
#define SC_LOW_V 4
#define SC_LOW_I 5
#define SC_LOW_RPM 6
#define SC_HIGH_V 7
#define SC_HIGH_I 8
#define SC_HIGH_RPM 9
#define SC_HIGH_WS 10
#define SC_HIGH_TP 11
const boolean GTEQ = true;   // used by threshold_checkers[]
const boolean LTEQ = false;  // used by threshold_checkers[]

// Create an array of threshold checkers.
// Call them individually as threshold_checkers[FURL_INIT_V], etc.
// This makes it easy to call them elsewhere - see checkFurlConditions(), checkSCSafeConditions(), checkSCNowConditions()
// We can do various things here, including checking and recording breadcrumbs.
thresholdChecker threshold_checkers[] = {
  // tail motor FURL thresholds. True if > threshold.
  thresholdChecker("", DC_VOLTAGE, &parm_furl_init_v, GTEQ, true),   // 0'th array element = FURL_INIT_V checker
  thresholdChecker("", DC_CURRENT, &parm_furl_init_i, GTEQ, true),   // 1 = FURL_INIT_I
  thresholdChecker("", RPM, &parm_furl_init_rpm, GTEQ, true),        // 2 = FURL_INIT_RPM
  thresholdChecker("", WINDSPEED, &parm_furl_init_ws, GTEQ, true),   // 3 = FURL_INIT_WS

  // shorting contactor EXERCISE thresholds. True if <= threshold.
  thresholdChecker("", DC_VOLTAGE, &parm_sc_exer_v, LTEQ, true),     // 4 = SC_LOW_V
  thresholdChecker("", DC_CURRENT, &parm_sc_exer_i, LTEQ, true),     // 5 = SC_LOW_I
  thresholdChecker("", RPM, &parm_sc_exer_rpm, LTEQ, true),          // 6 = SC_LOW_RPM

  // shorting contactor FAILSAFE thresholds. True if > threshold.
  thresholdChecker("SC_HIGH_V", DC_VOLTAGE, &parm_sc_furled_failsafe_v, GTEQ, true),  // 7 = SC_HIGH_V
  thresholdChecker("SC_HIGH_I", DC_CURRENT, &parm_sc_furled_failsafe_i, GTEQ, true),  // 8 = SC_HIGH_I
  thresholdChecker("SC_HIGH_RPM", RPM, &parm_sc_furled_failsafe_rpm, GTEQ, true),     // 9 = SC_HIGH_RPM
  thresholdChecker("SC_HIGH_WS", WINDSPEED, &parm_sc_emer_ws, GTEQ, true),            // 10 = SC_HIGH_WS
  thresholdChecker("SC_HIGH_TP", TAIL_POSITION, &parm_sc_failsafe_tp, GTEQ, true)     // 11 = SC_HIGH_TP
};

// This function shorts the shorting contactor.
// The shorting contactor is normally closed (N.C.) = shorted. Voltage must be applied to it to open it (unshorted).
// HIGH means send voltage to the shorting contactor, unshorting it. LOW releases the contactor, shorting it.
void initSC() {
  Serial << "furlctl: Initializing shorting contactor in CLOSED = SHORTED state.\n";
  digitalWriteDirect(SC_CTL_PIN, LOW);
  digitalWriteDirect(SC_ACTIVE_LED_PIN, LOW);
  sc_shorted = 1;
}


// ********** LINEAR ACTUATOR MOTOR CONTROL - furlctl() **********
// This must be enabled in wwe.ino by: #define ENABLE_LINEAR_ACTUATOR 
// In adc.ino, this function is called FURLCTL_PER_SEC times per second from readADCs() which is called by a timer interrupt.
// At startup, it initializes to the FURLING state, where it stays for 15 seconds, then moves to FURLED. 
// After 1 minute, lacking any furl trigger, it will unfurl. Furl conditions can lengthen this.
//
// There are 2 SSR's in the W5 turbine controller for furling and unfurling. Both relays must NEVER be on at the same time!
// They are controlled by FURL_CTL_PIN and UNFURL_CTL_PIN (see pindefs.h).
// To FURL:   digitalWrite(FURL_CTL_PIN, HIGH) and digitalWrite(UNFURL_CTL_PIN, LOW)
// To UNFURL: digitalWrite(FURL_CTL_PIN, LOW) and digitalWrite(UNFURL_CTL_PIN, HIGH)
//
// If furl_motor_on counter is >0, turn on FURL_CTL_PIN and turn off UNFURL_CTL_PIN, or vice-versa.
// At one second intervals, decrement furl_motor_on, but not below zero, and stay there until we reenter FURL 
// or UNFURL state, which re-initializes furl_motor_on to 15 sec, after which it decrements down to zero.
// FURL_STATE_1 LED will be ON when furling and will persist when furled
// FURL_STATE_1 LED will be OFF when unfurling and will persist when unfurled
// FURL_STATE_2 LED will be ON when furling or unfurling
// FURL_STATE_2 LED will be OFF when furled or unfurled

void furlctl(boolean even_second) {
                                                                   // if the motor is ON..., i.e.,
  if (--furl_motor_on > 0) {                                       // if furl_motor_counter > 0, // "--" prefix syntax decrements the counter BEFORE evaluation
                                                                   //   furl_state can be either 0 (UNFURL) or 2 (FURL), nothing else!
    digitalWriteDirect(FURL_STATE_1_LED_PIN, furl_state==UNFURL);  //   if furl_state is UNFURL, then FURL_STATE_1_LED_PIN is TRUE/1 --> LED OFF
    digitalWriteDirect(FURL_CTL_PIN, furl_state==FURL);            //   if furl_state is FURL, then FURL_CTL_PIN is TRUE/1 --> FURL power ON
    digitalWriteDirect(UNFURL_CTL_PIN, furl_state==UNFURL);        //   if furl_state is UNFURL, UNFURL_CTL_PIN is FALSE/0 --> UNFURL power OFF
                                                                   //   Everything reverses if furl_state is 0
  } else {                                                         // if furl_motor_on is <= 0
    furl_motor_on = 0;                                             //   re-zero counter
    digitalWriteDirect(FURL_CTL_PIN, LOW);                         //   turn FURL power OFF
    digitalWriteDirect(UNFURL_CTL_PIN, LOW);                       //   turn UNFURL power OFF
  }
  digitalWriteDirect(FURL_STATE_2_LED_PIN, !furl_motor_on);  // if we're furling OR unfurling, !furl_motor_on is FALSE/0 --> LED ON

  // Check furl conditions.
  last_furl_reason = furl_reason;       // save furl_reason into last_furl_reason
  furl_reason = checkFurlConditions();  // get latest furl_reason (if any): VOLT, CURR, RPM, WIND, EXER, ANEM, SLIP, MORN

  // Set furl duration.
  // quiet_time (interval between furl triggers) is managed by checkFurlConditions()
  if ( furl_reason && !last_furl_reason ) {                                             // if a furl condition has JUST occurred (i.e., last_furl_reason==0)...
    if ( quiet_time > 20*FURLCTL_PER_SEC ) {                                            //   if we're beyond a short interval where we ignore gusts...
      furl_duration = max(((10*60*FURLCTL_PER_SEC) - quiet_time), 60*FURLCTL_PER_SEC);  //     set furl duration to the greater of (10 min - quiet_time) or 1 min
    }
  }

  // If furl_reason > 0 (true), set unfurl_countdown = furl_duration.
  // If furl_reason==0 (false), decrement unfurl_countdown, but don't go below 0.
  unfurl_countdown = max( furl_reason ? furl_duration : unfurl_countdown - 1, 0);
  
  // NOTE: If shutdown_state is MANUALLY set to 1 when furl_state == UNFURL, 
  // then furl_reason (and furl_reason_saved) will be ZERO and we will not "see" a reason for the furl!
  // furl_state is initialized to FURL above.
  // shutdown_state is initialized to 0 UNLESS it has been set = 2.
  if ( furl_state==UNFURL ) {                     // if we're UNFURLed...
    if ( furl_reason || shutdown_state ) {        //   and if we need to furl or shutdown...
      furl_state = FURL;                          //     change furl_state to FURL=2
      furl_motor_on = FURL_RUN_TIME;              //     reset motor furl timer
      furl_reason_saved = furl_reason;            //     save furl_reason (for debug)
    } else {
      // not conditions for furl.
    }     
  } else {                                         // if we're FURLed...
    if ( !unfurl_countdown && !shutdown_state ) {  //   and we're not counting down or in shutdown
      furl_state = UNFURL;                         //     change furl_state to UNFURL=0
      furl_motor_on = FURL_RUN_TIME;               //     reset motor furl timer
      furl_reason_saved = 0;                       //     reset furl_reason_saved
      engage_sc = false;                           //     reset flag
    }
    // Check for failsafe conditions while FURLed...
    //   While failsafe conditions persist, we wait sc_failsafe_countdown iterations before shorting.
    //   If sc_failsafe_reason > 0 (true), decrement sc_failsafe_countdown, but not < 0.
    //   If sc_failsafe_reason==0 (false), reset sc_failsafe_countdown to 30*FURLCTL_PER_SEC
    sc_failsafe_reason = checkSCNowConditions();
    sc_failsafe_countdown = max( (sc_failsafe_reason ? (sc_failsafe_countdown - 1) : 30*FURLCTL_PER_SEC), 0 );

    // Set shutdown_state=2 if failsafe countdown timer reaches 0.
    if ( (sc_failsafe_countdown == 0) && (shutdown_state != 2) ) {
      shutdown_state = 2;                   // set hard shutdown state
      parm_shutdown_state.setParmVal("2");  // write parm to SD so this state PERSISTS ACROSS RESETS
    }
  }  // END if we're FURLed else{}

  // Finally, manage the shorting contactor based on sc conditions and shutdown state
  // NOTE: engage_sc is set in checkFurlConditions()
  // sc_shorted  > 0 == TRUE  == SC is shorted
  // sc_shorted == 0 == FALSE == SC is unshorted
  // SHORT if (we're in Manual Mode AND Manual SC switch is ON) OR
  //          (we're not in Manual Mode AND ( SS is 2 OR 
  //                                         (SS is 1 and it's safe to short) OR
  //                                         (it's an exercise furl and it's safe to short) ) )
  sc_shorted = ( checkManualMode() && checkManualSCShort() ) ||
               ( !checkManualMode() && ( shutdown_state == 2 || 
                                        (shutdown_state == 1 && checkSCSafeConditions()) ||
                                        (engage_sc           && checkSCSafeConditions()) ) );

  // Reverse the logic for direct SSR control because...
  // writing NON-ZERO to SC_CTL_PIN turns SSR on, which UNSHORTS the SC.
  // writing ZERO to SC_CTL_PIN turns SSR off, which SHORTS the SC.
  digitalWriteDirect(SC_CTL_PIN, !sc_shorted);         // SC_CTL_PIN = 0 = shorted
  digitalWriteDirect(SC_ACTIVE_LED_PIN, !sc_shorted);  // SC_ACTIVE_LED_PIN = 0 = LED on

}
// ********** end LINEAR ACTUATOR MOTOR CONTROL - furlctl() **********





// ********** STEPPER MOTOR CONTROL - furlctl1() **********
// Stepper motor control must be enabled in wwe.ino by: #define ENABLE_STEPPER 
// This function is called FURLCTL1_PER_SEC times per second from readADCs() which is called by a timer interrupt (see adc.ino)
//
// Anaheim Automation stepper motor + MBC12101 driver + slewing drive notes:
// 360 deg/rev / 1.8 deg/step = 200 steps/rev * 10 usteps/step = 2000 usteps/rev
// Slewing drive has 62 input revs per 1 output rev --> 62:1 gear ratio.
// So, there are 62 * 2000 / 360 = 344.44 usteps per deg of slewing drive motion.
//
// For minimum control complexity, we simply specify the desired position. The stepper control accelerates 
// the motor to the max velocity, holds that velocity as long as necessary, then decelerates and stops.
// If desired, motor acceleration and max velocity can be changed to adapt to conditions.
//
// The following stepper motor control (setter and getter) functions are called, for example, as:
// motor.currentPosition(int) - setter function (integer arg)
// motor.currentPosition() - getter function (no arg)
//
// void Acceleration(int) - set acceleration, usteps per second per call of motor state control
// int Acceleration() - get acceleration
//
// void maxVelocity(int) - set max velocity (usteps/s)
// int maxVelocity() - get max velocity
//
// void currentVelocity(int) - set current velocity (usteps/s)
// int currentVelocity() - get current velocity
//
// void currentPosition(int) - set current position (usteps)
// int currentPosition() - get current position
//
// void desiredPosition(int) - set desired position (usteps)
// int desiredPosition() - get desired position
//
// getChannelRMSInt() returns current value of the selected channel as an integer, e.g., getChannelRMSInt(DC_VOLTAGE),
// where the following const's match channel names to int channel numbers.
// const int L1_VOLTAGE = 0;
// const int L2_VOLTAGE = 1;
// const int L3_VOLTAGE = 2;
// const int DC_VOLTAGE = 3;
// const int LINE_VOLTAGE = 4;
// const int L1_CURRENT = 5;
// const int L2_CURRENT = 6;
// const int L3_CURRENT = 7;
// const int DC_CURRENT = 8;
// const int L1L2_VOLTAGE = 9;
// const int L2L3_VOLTAGE = 10;
// const int L3L1_VOLTAGE = 11;
// const int RPM = 12;
// const int WINDSPEED = 13;
// const int DUMP_LOAD_P = 14;
// const int RECT_TEMP = 15;
// const int AMBIENT_TEMP = 16;
// const int CONTROLLER_TEMP = 17;
// const int TAIL_POSITION = 18;



void furlctl1() {
  
  // LOCAL VARS
  static boolean manual_furl_state = false;                       // true when manually furling, motor moving (+)
  static boolean last_manual_furl_state = manual_furl_state;
  static boolean manual_unfurl_state = false;                     // true when manually unfurling, motor moving (-)
  static boolean last_manual_unfurl_state = manual_unfurl_state;
  static boolean reed_switch_state = !debounced_rs_state;         // 0=open, 1=closed
  static boolean last_reed_switch_state = reed_switch_state;

  static boolean motor_zero_flag = 0;   // = 1 when motor position == 0 == straight back, unfurled
  static int furlDir = 1;               // flag toggles between +1 and -1 
  int motorVelocity = 0;                // absolute value of slew rate; sets maxVelocity
  int motorDPS = 0;                     // motor velocity, degrees per second
  static int stepper_furl_timer = 0;    // iteration counter for FULL furl (V,I,RPM,WS)
  static int full_furl_time = 0;        // iterations to remain fully furled (EX differs from V,I,RPM,WS)
  static boolean initialize_TP = true;
  static int target_motor_position = motor.currentPosition();
  static int tp = 0;                    // TP initialization counter
  static int tp_search_timer = 0;       // timer for (1 second) pause between TP search directions

  // Vars used in frozen anemometer detection.
  static int RPM_nonzero = 0;  // non-zero RPM counter
  static int WS_nonzero = 0;   // non-zero WS counter
  static int WS_zero = 0;      // zero WS counter

  // These arrays hold a moving window of RPM and WS observations to be averaged for furl control.
  // Change # of elements in the arrays below to change the width of the averaging window.
  // For C++ array initialization, see https://www.cplusplus.com/doc/tutorial/arrays/
  static int rpm_arr[5] = { };                         // # of array elements initialized here SETS the # of RPM obs to average
  const int rpm_arrlen = sizeof(rpm_arr)/sizeof(int);  // # of elements in RPM array
  int rpm_arrsum = 0;                                  // sum of elements in RPM array, not static
  static int rpm_avg = 0;                              // RPM (rpm, 1024x actual)
  static int last_rpm_avg = 0;                         // saved RPM (rpm, 1024x actual)
  int dRPMdt = 0;                                      // RPM derivative (RPM per *second*, 1024x actual)
  int predRPM = 0;                                     // predicted RPM (1024x actual)

  static int ws_arr[600] = { };                        // # of array elements initialized here (to zero) = # of 1-sec WS obs to average
  const int ws_arrlen = sizeof(ws_arr)/sizeof(int);    // # of elements in WS array
  int ws_arrsum = 0;                                   // sum of elements in WS array, not static
  static int ws_arrsize = 0;                           // length of ws_arr[]
  static int ave_count = 0;                            // # of 1-sec WS obs
  static int ws_max = 0;                               // max WS in ws_arr[]
  static int ws_avg = 0;                               // average WS of ws_arr[] (m/s, 1024x actual)
  static int last_ws_avg = 0;                          // saved WS (m/s, 1024x actual)
  static int ws_mad = 0;                               // mean absolute deviation (MAD) of average WS of ws_arr[] (m/s, 1024x actual)
  int dWSdt = 0;                                       // WS derivative (m/s per *second*, 1024x actual)

  // These vars need to be static because we use them *every* furlctl iteration, not just when they're computed once-per-second!
  static float rayleigh[41] = { };      // Rayleigh wind speed probability distribution (0-40 mph, 1 mph bins)
  static float rayleigh_sum[41] = { };  // sum of (1-rayleigh[i]) = probability of all wind speeds >= index
  static int hold_WSgte[41] = { };      // array of TP hold times at wind speed == index, computed from rayleigh_sum[] vals
  static int latest_WSgte[41] = { };    // array of latest unixtimes when wind speed >= index
  static boolean WSgte[41] = { };       // array of booleans when wind speed >= index
  static int TP_remaining = 0;          // TP hold time remaining (seconds).

  int predTP = 0;                      // predicted tail position (usteps)
  static int predTP_saved = 0;         // saved predTP (usteps). Static!
  static int TP_timer = 10000;         // tail position timer. Initialize to > desired # of furlctl1() iterations. Static!

  int current_motor_position = 0;      // not static
  static int last_motor_position = 0;  // saved current_motor_position. Static!
  static int total_tail_motion = 0;    // cumulative angular motion of tail. Static!

  // end LOCAL VARS



  // Stepper motor acceleration. This should NOT need to be changed below this.
  motor.Acceleration(5);  // 5 usteps/s/(machine state eval)*(1000 evals/s) = 5000 usteps/s/s

  // Logic board LED's  
  if ( motor.isMotorOn() ) {  // if stepper motor is ON...
    // turn FURL_STATE_1 LED ON if motion is (+), otherwise turn FURL_STATE_2 LED ON
    motor.motorDir() ? digitalWriteDirect(FURL_STATE_1_LED_PIN, 0) : digitalWriteDirect(FURL_STATE_2_LED_PIN, 0);
  } else {                    // otherwise...
    digitalWriteDirect(FURL_STATE_1_LED_PIN, 1);  // turn both LED's OFF
    digitalWriteDirect(FURL_STATE_2_LED_PIN, 1);
  }

  // Below, we do the following (reminder... at a rate of FURLCTL1_PER_SEC):
  // CHECK REED SWITCH         - allowed unconditionally - watches and recalibrates tail position (TP)
  // CHECK FAILSAFES           - allowed unconditionally - sets (but does not unset) shutdown_state=2
  // MANAGE SHORTING CONTACTOR - allowed unconditionally - sets/unsets shorting contactor
  // MANUAL MODE               - allowed if: Manual Mode true (physical switch) - sets initialize_TP=true
  // INITIALIZE TAIL POSITION  - allowed if: Manual Mode false AND initialize_TP true (shutdown_state can be anything)
  // FURL IF SHUTDOWN STATE    - allowed if: Manual Mode false AND initialize_TP false AND shutdown_state true
  // AUTO MODE                 - allowed if: Manual Mode false AND initialize_TP false AND shutdown_state false
  //
  // Operating scenarios:
  // Restarting with Manual Mode on:  NOTHING HAPPENS, we just stay in Manual Mode. 
  //                                  If shutdown_state!=2, shutdown_state is set to 1 - by setup().
  // Restarting with Manual Mode off: TP initializes (ending furled). 
  //                                  If shutdown_state!=2, shutdown_state is set to 1 - by setup() and on exit from initialize_TP.
  // Exiting Manual Mode at any time: TP initializes (ending furled). 
  //                                  If shutdown_state!=2, shutdown_state is set to 1 - on exit from initialize_TP.

  // ********** CHECK REED SWITCH **********
  // Recalibrate/reset tail position when RS closes.
  last_reed_switch_state = reed_switch_state;     // save last state
  reed_switch_state = !debounced_rs_state;        // get new state

  if ( (reed_switch_state == 1) && (last_reed_switch_state == 0) ) {  // if RS has JUST CLOSED...
    if ( motor.currentPosition() >= 0 ) {                             //   if current TP is (+)
      motor.currentPosition(RSlimit);                                 //     reset motor position to (+)LIMIT
      motor.desiredPosition( motor.currentPosition() );               //     STOP MOTOR - because we've reached a limit!
    } else {                                                          //   otherwise, TP is (-)
      motor.currentPosition(-RSlimit);                                //     reset motor position to (-)LIMIT
      motor.desiredPosition( motor.currentPosition() );               //     STOP MOTOR - because we've reached a limit!
    }
  }
  // ********** end CHECK REED SWITCH **********  


  // ********** CHECK SC FAILSAFES **********
  // First, check ALL shorting contactor short conditions
  sc_reason = checkSCConditions();                   // this resets sc_reason EVERY iteration
  if ( sc_reason > 0 ) sc_reason_saved = sc_reason;  // save sc_reason, used by getControllerState(), global var

  sc_failsafe_reason = checkSCNowConditions();  //   check failsafe conditions (VOLT, CURR, RPM, TAIL, TPINIT), global var
  if ( sc_failsafe_reason ) {                   //   if we find ANY failsafe conditions...
    shutdown_state = 2;                         //     set hard shutdown_state
    parm_shutdown_state.setParmVal("2");        //     write state to SD so it PERSISTS ACROSS RESETS
                                                //     The only way to exit SS2 is by setting SS1 or SS0 on the parms page!
                                                //     If SS2 is caused by tp_init_fail, it can only be cleared by entering Manual Mode, 
                                                //     and then setting SS1 or SS0 on the parms page.
    stepper_furl_timer = 0;                     //     Rezero this; otherwise, we'll wait full_furl_time before unfurling!
  }
  // ********** end CHECK FAILSAFES **********


  // ********** MANAGE SHORTING CONTACTOR **********
  // NOTE: engage_sc is set when furl_reason is evaluated in the switch() statement below.
  // sc_shorted  > 0 == TRUE  == SC is shorted
  // sc_shorted == 0 == FALSE == SC is unshorted
  // SHORT if (we're in Manual Mode AND Manual SC switch is ON) OR
  //          (we're not in Manual Mode AND (we're initializing tail position OR
  //                                         SS is 2 OR 
  //                                        (SS is 1 and it's safe to short) OR
  //                                        (engage_sc has been set and it's safe to short) )
  sc_shorted = ( checkManualMode() && checkManualSCShort() ) ||
               ( !checkManualMode() && (initialize_TP || 
                                        shutdown_state == 2 || 
                                       (shutdown_state == 1 && checkSCSafeConditions()) ||
                                       (engage_sc           && checkSCSafeConditions()) ) );
                                       
  // We're always shorted when initialize_TP==true, because shutdown_state=1 pertains during startup
  // Reverse the logic for direct SSR control because...
  // writing NON-ZERO to SC_CTL_PIN turns SSR on, which UNSHORTS the SC.
  // writing ZERO to SC_CTL_PIN turns SSR off, which SHORTS the SC.
  digitalWriteDirect(SC_CTL_PIN, !sc_shorted);         // SC_CTL_PIN = 0 = shorted
  digitalWriteDirect(SC_ACTIVE_LED_PIN, !sc_shorted);  // SC_ACTIVE_LED_PIN = 0 = LED on
  // ********** end MANAGE SHORTING CONTACTOR **********

  // ********** MANUAL MODE **********
  // Entering Manual Mode is possible ONLY via the physical switch on the controller!
  // Auto Mode (see code below), TP initialization (see code above), and furling in shutdown_state=1 or 2 (see code below)...
  // ...all of these CHECK that we're not in Manual Mode!
  // As such, there are no checks to enter Manual Mode other than the physical switch!
  //

  if ( checkManualMode() ) {
    last_manual_furl_state = manual_furl_state;     // save last manual furl (+) state
    manual_furl_state = checkManualFurl();          // get new manual furl (+) state
    last_manual_unfurl_state = manual_unfurl_state; // save last manual unfurl (-) state
    manual_unfurl_state = checkManualUnfurl();      // get new manual unfurl (-) state

    initialize_TP = true;                             // initialize TP on exiting from MM
    tp_init_fail = false;                             // reset TP initialization fail flag
    target_motor_position = motor.currentPosition();  // used in initialize TP, on exiting from MM
    total_tail_motion = 0;                            // re-zero because we've exited AUTO mode
    stepper_furl_timer = 0;                           // re-zero because we've exited AUTO mode
    furl_reason_saved = 0;                            // re-zero because we've exited AUTO mode
    sc_reason_saved = 0;                              // re-zero because we've exited AUTO mode

    // Stop motor if no motor command is active.
    // We use _last values because we handle motor deceleration below when manual_(un)furl_state goes false.
    if ( !last_manual_furl_state && !last_manual_unfurl_state ) {
      motor.desiredPosition(motor.currentPosition());
    }
    
    // If manual_furl_state == true --> move motor (+)
    if (manual_furl_state) {
      if (reed_switch_state == 0) {              // If RS is OFF...
        motorDPS = 5;                            // 5 deg/s. No faster because if RS has failed... see next comments.
        motorVelocity = (motorDPS*2000*62)/360;  // usteps/s
        motor.maxVelocity(motorVelocity);
      }
      if (reed_switch_state == 1) {              // Move SLOWLY if RS is ON. This discourages, but WILL NOT PREVENT, the operator from crashing the tail.
        motorDPS = 1;                            // 1 deg/s
        motorVelocity = (motorDPS*2000*62)/360;  // usteps/s
        motor.maxVelocity(motorVelocity);        
      }
      motor.desiredPosition(motor.currentPosition() + 10000);
      //motor.desiredPosition(1000000);  // choose a very large (+). This didn't improve noise problem.
    } else {  // this happens the moment we stop furling (manual_furl_state == 0), but last_manual_furl_state still == 1.
      if (last_manual_furl_state) {
        motor.desiredPosition( motor.currentPosition() + motor.decelX() );  // decelerate to final position
        //motor.desiredPosition( motor.currentPosition() );  // Nor did this improve noise problem.
      }
    }
    // If manual_UNfurl_state == true --> move motor (-)
    if (manual_unfurl_state) {
      if (reed_switch_state == 0) {              // If RS is OFF...
        motorDPS = 5;                            // 5 deg/s. No faster because if RS has failed... see next comments.
        motorVelocity = (motorDPS*2000*62)/360;  // usteps/s
        motor.maxVelocity(motorVelocity);
      }
      if (reed_switch_state == 1) {              // Move SLOWLY if RS is ON. This discourages, but WILL NOT PREVENT, the operator from crashing the tail.
        motorDPS = 1;                            // 1 deg/s
        motorVelocity = (motorDPS*2000*62)/360;  // usteps/s
        motor.maxVelocity(motorVelocity);        
      }
      motor.desiredPosition(motor.currentPosition() - 10000);
      //motor.desiredPosition(-1000000);  // choose a very large (-). This didn't improve noise problem.
    } else {  // this happens the moment we stop unfurling (manual_unfurl_state == 0), but last_manual_unfurl_state still == 1.
      if (last_manual_unfurl_state) {
        motor.desiredPosition( motor.currentPosition() - motor.decelX() );  // decelerate to final position
        //motor.desiredPosition( motor.currentPosition() );  // Nor did this improve noise problem.
      }
    }

    // ***PROCESS SERIAL MONITOR INPUT*** for Manual Mode tail positioning
    static int serialInt = 0;   // serial input = desired tail angle (Manual Mode only): -90 deg to +90 deg
    static int serialFlag = 0;  // serial input flag (Manual Mode only)
    if (Serial.available() > 0) {     // interpret Serial input as desired tail angle in +/-deg units
      serialInt = Serial.parseInt();  // parseInt() disallows non-integer input
      if ( abs(serialInt) > 85 ) {    // disallow tail positions > +/-85 deg
        serialInt = 0;
        serialFlag = 0;
      } else {
        serialFlag = 1;
      }
    }
    if ( serialFlag == 1 ) {          // respond to Serial input
      motor.desiredPosition( serialInt*2000*62/360 );
      serialFlag = 0;                 // reset serialFlag; ensures this if() excutes only when a NEW serialInt is available
    }
    
  }
  // ********** end MANUAL MODE **********

  // ********** INITIALIZE TAIL POSITION **********
  // Initialize TP ONLY if we're NOT in Manual Mode.
  // initialize_TP is true:
  //   on controller restart (normally into shutdown_state 1, but could be 2), OR
  //   after a firmware update, OR 
  //   on exit from Manual Mode
  if ( initialize_TP && !checkManualMode() ) {

    // We reset these here so that the STATE bitfield is unchanged UNTIL we've reset TP (e.g., if SS=2 is triggered by TP).
    furl_reason_saved = 0;  
    sc_reason_saved = 0; 
    total_tail_motion = 0;  // reset because we're re-initializing TP

    // We decide what to do here based on reed_switch_state and last_reed_switch_state. 
    // There are 4 POSSIBILITIES, each addressed with its own code below...
    //
    // If (1,1), we don't know if the tail is at its (+) or (-) limit. This is the most difficult case.
    // So, find it by searching SLOWLY (1 deg/s) within a restricted search window.
    if ( (reed_switch_state == 1) && (last_reed_switch_state == 1) ) {
      motorDPS = 1;                                  // move tail at 1 deg/s
      motorVelocity = (motorDPS*2000*62)/360;        // usteps/s
      motor.maxVelocity(motorVelocity);
      motor.desiredPosition(target_motor_position);  // at startup, target_motor_position = motor.currentPosition()
      // If we reach the target position, but RS is still closed, reverse search direction and change target position
      // The tp sequence 1..10 (tp++) yields search positions (deg): -1, +1, -2, +2, -3, +3, -4, +4, -5, +5
      // The tp sequence 2..20 (tp+=2) yields search positions (deg): -2, +2, -4, +4, -6, +6, -8, +8, -10, +10
      // The tp sequence 5..20 (tp+=5) yields search positions (deg): -5, +5, -10, +10
      if ( motor.currentPosition() == target_motor_position ) {
        // The tp_search_timer allows for delayed reed switch activation before we reverse search direction. 
        // If we don't do this, motorDir() may report incorrect direction resulting in setting wrong sign RSlimit in next state (0,1).
        tp_search_timer++;
        if ( tp_search_timer > (2*FURLCTL1_PER_SEC) ) {  // add a 2 second pause between direction reversals
          tp_search_timer = 0;
          tp += 5;                  // widen search window by 5 deg; initialized to tp = 0
          if (tp > 20) {            // if this is true, our search has FAILED, so...
            tp = 0;                 //   rezero counter
            if (motor.motorDir() == 1) target_motor_position = motor.currentPosition() - 10*344;  // restore pre-search TP
            if (motor.motorDir() == 0) target_motor_position = motor.currentPosition() + 10*344;
            tp_init_fail = true;    //   set flag - used in checkSCNowConditions() below
            initialize_TP = false;  //   ESCAPE from initialize TP routine
          } else {
            if (motor.motorDir() == 1) target_motor_position = motor.currentPosition() - tp*344;  // if last motor dir was (+), search (-)
            if (motor.motorDir() == 0) target_motor_position = motor.currentPosition() + tp*344;  // if last motor dir was (-), search (+)
          }
        }
      }
    }
    // If (0,1), then RS state has JUST changed to 0 (we escaped from RS on!), so set current position.
    // This state only lasts ONE ITERATION before changing to the (0,0) state below.
    if ( (reed_switch_state == 0) && (last_reed_switch_state == 1) ) {
      // If motorDir is 1(+), then we must have been at -RSlimit; otherwise, the opposite.
      if ( motor.motorDir() == 1 ) motor.currentPosition(-RSlimit);
      if ( motor.motorDir() == 0 ) motor.currentPosition(RSlimit);
    }
    // If (0,0), the tail could be ANYWHERE between (+) and (-) limit *OR* we just escaped from RS on (the case above).
    // If the former, we should move *slowly* because we *could* be very close to RSlimit already.
    // If the latter, we *know* we are very close to RSlimit and need to move *slowly* back to it.
    // This will be the typical state when exiting Manual Mode, hopefully with a fairly accurate TP set by manual RS trigger(s).
    // Regardless, in this state, we can do no better than ASSUME that TP is fairly accurate, but we move *slowly* to closest limit.
    if ( (reed_switch_state == 0) && (last_reed_switch_state == 0) ) {
      motorDPS = 1;                                                       // FURL @ 1 deg/s
      motorVelocity = (motorDPS*2000*62)/360;                             // usteps/s
      motor.maxVelocity(motorVelocity);
      if ( abs(motor.currentPosition()) < FURLlimit ) {  // move tail ONLY if we're within allowable limits
        if ( motor.currentPosition() >= 0 ) motor.desiredPosition(FURLlimit);
        if ( motor.currentPosition() < 0 ) motor.desiredPosition(-FURLlimit);
      }
    }
    // If (1,0), we've just fully furled normally, and TP recalibrated (when RS changed state).
    // So, we can ESCAPE from TP initialization here, 
    if ( (reed_switch_state == 1) && (last_reed_switch_state == 0) ) {
      initialize_TP = false;
      tp = 0;                                     // rezero TP search angle counter
      tp_search_timer = 0;                        // rezero TP search timer 
      if ( parm_shutdown_state.intVal() != 2 ) {  // if we're not in SS2...
        parm_shutdown_state.setParmVal("1");      //   put turbine in SS1
        shutdown_state = 1;
      }
    }
  }
  // ********** end INITIALIZE TAIL POSITION **********

  // ********** FURL IF SHUTDOWN STATE **********
  // DURING initialize_TP, or while in Manual Mode, NOTHING HAPPENS due to !initialize_TP flag.
  // AFTER initialize_TP, reed_switch_state will ==1, again NOTHING HAPPENS because we're already furled.
  // ONLY if we're in Auto Mode will we FURL in response to setting shutdown_state 1 or 2.
  //
  if ( (shutdown_state > 0) && !initialize_TP && !checkManualMode() ) {

    // Reset these variables if SS=1 or =2
    TP_timer = 10000;        // re-initialize tail position timer
    total_tail_motion = 0;   // reset cumulative tail motion
    stepper_furl_timer = 0;  // reset full furl timer
    full_furl_time = 0;      // reset full furl time
    engage_sc = false;       // because setting SS=1 or =2 closes the SC, we can reset this flag which may have been set true by various furl triggers

    if ( shutdown_state == 1 ) {  // if we've (manually) set SS=1 from SS=2 (or SS=0)...
      furl_reason_saved = 0;      //   we no longer need to see the furl reason
      sc_reason_saved = 0;        //   we no longer need to see the SC reason
    }
    
    if ( abs(motor.currentPosition()) >= (RSlimit - 3444) ) {  // if we're close to RSlimit
      motorDPS = 1;                                            //   FURL SLOWLY to avoid overshoot
      motorVelocity = (motorDPS*2000*62)/360;                  //   deg/s to usteps/s
      motor.maxVelocity(motorVelocity);
    } else {                                                   // otherwise,
      motorDPS = 20;                                           //   FURL QUICKLY
      motorVelocity = (motorDPS*2000*62)/360;                  //   deg/s to usteps/s
      motor.maxVelocity(motorVelocity);
    }
    if (reed_switch_state != 1) {  // FURL until RS closes or FURLlimit is reached
      if ( abs(motor.currentPosition()) < FURLlimit ) {  // move tail ONLY if we're within allowable limits
        if ( motor.currentPosition() >= 0 ) motor.desiredPosition(FURLlimit);
        if ( motor.currentPosition() < 0 ) motor.desiredPosition(-FURLlimit);
      }
    }
  }  
  // ********** end FURL IF SHUTDOWN STATE  **********


  // ********** COLLECT RPM AND WS MEASUREMENTS **********
  // We can do this safely outside of any furl control "state"
  //
  // Collect RPM observation sums and calculate averages:
  // Array length==5 and FURLCTL1_PER_SEC==10 gives 0.5 sec averaging time. See dRPM/dt calculation below.
  // EVERY furlctl1() iteration, the array loses its 0th element, and rpm_avg is updated.
  
  rpm_arrsum = 0;                                 // re-zero RPM sum
  for (int i = 1; i < rpm_arrlen; i++) {          // loop over RPM array elements...
    rpm_arr[i-1] = rpm_arr[i];                    //   shift WS array elements, 0<--1, 1<--2, 2<--3, 3<--4
    rpm_arrsum += rpm_arr[i-1];                   //   accumulate sum of shifted RPM array elements
  }
  // Write latest obs to the last array element, e.g., rpm_arr[(rpm_arrlen-1].
  // The new average is computed from the sum after saving the last one.
  rpm_arr[rpm_arrlen-1] = getChannelRMSInt(RPM);  // put latest RPM obs into array as its last element (rpm, 1024x actual)
  rpm_arrsum += rpm_arr[rpm_arrlen-1];            // add it to the sum (rpm, 1024x actual)
  last_rpm_avg = rpm_avg;                         // save old ave RPM (rpm, 1024x actual)
  rpm_avg = rpm_arrsum / rpm_arrlen;              // compute new ave RPM (rpm, 1024x actual)
  
  // If we don't average and just use the lines below, we get way too much rpm-trigerred furling!
  //last_rpm_avg = rpm_avg;
  //rpm_avg = getChannelRMSInt(RPM);

  // copy some local vars to global ones for *slow* printing in loop() - see wwe.ino
  //print_rpm_arrlen = rpm_arrlen;
  //print_rpm_arrsum = rpm_arrsum/1024.;
  //print_rpm_avg = rpm_avg/1024.;

  // FOR TESTING: put a frequency channel val into the LV channel so we can look at it in the web monitor
  //              For instantaneous val (poss. median-filtered, poss. rectified), use method getInstantaneousValInt()
  //              For time-averaged val, use method getAvgValInt()
  //analog_channels[LINE_VOLTAGE].setInstantaneousValInt( ac_rpm.getInstantaneousValInt(), false, false, false );  // RPM from V1,V2,V3 average
  //analog_channels[LINE_VOLTAGE].setInstantaneousValInt( ac_pll.getInstantaneousValInt(), false, false, false );  // RPM from L1L2 diff channel

    
  // Collect WS observation sums and calculate average and mean absolute deviation (MAD).
  // We use these data below to set TP hold time thresholds in AUTO mode state.
  // Because new WS data arrive slowly and irregularly (every 0.5 to 1 sec), we don't need to sample WS at *every* iteration of furlctl1().
  // So, we set up ws_arr[] to hold ws_arrlen values sampled at *exactly* 1-sec intervals, i.e., FURLCTL1_PER_SEC.
  // Every FURLCTL1_PER_SEC iterations, the WS array loses its 0th element.
  // ws_arrlen (const int defined above) determines the interval over which average WS is calculated.
  ave_count++;
  if (ave_count == FURLCTL1_PER_SEC) {                               // At 1-sec intervals...
    ave_count = 0;                                                   //   rezero averaging counter
    ws_arrsize++;                                                    //   increment WS array size
    if (ws_arrsize > ws_arrlen) {                                    //   if we've reached max array length...
      ws_arrsize = ws_arrlen;                                        //     constrain ws_arrsize to ws_arrlen
      for (int i = 1; i < ws_arrsize; i++) ws_arr[i-1] = ws_arr[i];  //     shift WS array elements, 0<--1, 1<--2 ... 598<--599
    }
    ws_arr[ws_arrsize-1] = getChannelRMSInt(WINDSPEED);              //   put latest WS into last array element (m/s/, 1024x actual)
    ws_arrsum = 0;                                                   //   rezero WS array sum
    for (int i = 0; i < ws_arrsize; i++) ws_arrsum += ws_arr[i];     //   compute sum of array vals
    last_ws_avg = ws_avg;                                            //   save old ave WS (m/s, 1024x actual)
    ws_avg = ws_arrsum / ws_arrsize;                                 //   compute new ave WS (m/s, 1024x actual)
    ws_max = 0;                                                      //   rezero ws_max
    for (int i = 0; i < ws_arrsize; i++) {
      if ( ws_arr[i] > ws_max ) ws_max = ws_arr[i];                  //   find the max array val (m/s, 1024x actual)
    }
      
    ws_mad = 0;                                                            // rezero MAD
    for (int i=0; i < ws_arrsize; i++) ws_mad += abs(ws_arr[i] - ws_avg);  // compute MAD sum, 1024x actual
    ws_mad = ws_mad / ws_arrsize;                                          // compute MAD, 1024x actual

    // Use ac_Pd as a TEMPORARY parm for monitoring |WS|
    //ac_Pd.setInstantaneousValInt( ws_avg, false, false, false );     // m/s, 1024x actual; DON'T median, DON'T rectify, DON'T filter

    // Use LINE_VOLTAGE as a TEMPORARY parm for monitoring |WS|+3*MAD
    // To allow this, the analog channel read for LINE_VOLTAGE is DISABLED! - see adc.ino
    //analog_channels[LINE_VOLTAGE].setInstantaneousValInt( ((ws_avg + 3*ws_mad)*MS2MPH), false, false, false );  // 1024x actual, DON'T median, DON'T rectify, DON'T filter

    // Compute latest_WSgte[ws] array elements = unixtime when WS >= ws.
    // These vals are used below to set the TP hold time as a function of WS.
    // ws_arr[ws_arrsize-1] is the last element of an array which is updated every SECOND (not at FURLCTL1_PER_SEC).
    // This is the WS value we want to know for this calculation.
    // This could be a useful array for other purposes, so we compute for all WS from 0-40 --> ws++
    for (int ws = 0; ws < 41; ws++) {
      if ( ws_arr[ws_arrsize-1] >= (ws*MPH2MS*1024) ) latest_WSgte[ws] = myunixtime;
    }

    // Compute the Rayleigh probability distribution from the average WS.
    // The Rayleigh distribution equation used below is taken from _Wind Power_ (Paul Gipe, 2004), p37.
    // We need the *entire* distribution to get cumulative probabilities in rayleigh_sum[].
    // The latter is used below to compute TP hold times. 
    // ***This should probably be converted to INTEGER math to minimize processing time. Arghhh!***
    float ws_avg_mph = ws_avg*MS2MPH/1024.;  // FLOATING POINT! convert (1024*m/s) to mph
    if (ws_avg_mph > 1) {                    // if() avoids divide by zero
      for (int ws = 0; ws < 41; ws++) {      // compute *entire* distribution ws = 0 to 40 mph
        rayleigh[ws] = HALF_PI * ws/sq(ws_avg_mph) * exp( -(PI/4)*sq(ws/ws_avg_mph) );  // floating point math! int/float = float
        if (ws == 0) rayleigh_sum[0] = 1;
        if (ws > 0) rayleigh_sum[ws] = rayleigh_sum[ws-1] - rayleigh[ws];  // rayleigh_sum[ws] = probability of all wind speeds >= ws
      }
    }

    // Now, using the cumulative WS probabilities, we compute TP hold times for ws >= 22 mph.
    // We do this based on two assumptions:
    //   1. We ASSUME (based on observation) that for ws < 22 mph, NO furl is required, and conversely, that furling IS required for ws >= 22 mph.
    //      Note that this threshold may change with, for example, a new blade set with different performance!
    //      The hold time (in seconds) for ws==22 mph is MUCH GREATER THAN hold times calculated in the following loop.
    //      This is intended to compensate for a too-slow tail slewing response from TP=0 in turbulent wind conditions.
    //      Specifically, holding TP at ~30 deg saves a critical 1.5 seconds of furling time (@20 deg/s) while
    //      sacrificing only 1-cos(30) = 0.13 = 13% of power. Theoretically.
    //   2. We ASSUME that cumulative Rayleigh probabilities are related to TP hold times using the ratio:
    //      Prob(WS >= ws) / Prob(WS >= 22), where ws >= 22 mph, is a factor by which TP hold times can be reduced as WS increases.
    //      This assumption is entirely empirical! ...but it seems to work.
    //      Note that the Rayleigh ratios vary significantly as the average WS changes... because the Rayleigh distribution changes, i.e.,
    //      higher average WS --> longer hold times at each WS threshold. This is what we want.
    //      Also, for wind speeds > 22 mph, TP hold times are scaled to 300 seconds, not 1200 seconds (ws==22 mph is a special case).
    //
    //      Loop over only those wind speeds that are needed for the TP calculation below --> *** ws+=2 ***
    hold_WSgte[22] = 1200;
    for (int ws = 24; ws < 41; ws+=2) {
      if (rayleigh_sum[22] > 0) hold_WSgte[ws] = max( (int)(300 * rayleigh_sum[ws]/rayleigh_sum[22]), 30 );  // if() avoids divide by zero, enforce 30-sec minimum
    }

    // TAIL POSITIONING:
    // Caculate the WSgte[ws] flags which are used below (along with predicted RPM) to dynamically set the TP.
    // If the elapsed time since a given WS was observed is < the TP hold time calculated for that WS, set flag WSgte[ws] = true, otherwise = false.
    // TP_remaining = hold time calculated for the *highest* WS that has been exceeded - time elapsed since that WS was observed.
    //
    // Loop over only those wind speeds that are needed for the TP calculation below --> *** ws+=2 ***
    for (int ws = 22; ws < 41; ws+=2) {  
      if ( (myunixtime - latest_WSgte[ws]) < hold_WSgte[ws] ) {
        WSgte[ws] = true;
        TP_remaining = hold_WSgte[ws] - (myunixtime - latest_WSgte[ws]);
      } else {
        WSgte[ws] = false;
      }
    }
    if ( !WSgte[22] ) TP_remaining = 0;  // if WS has not exceeded 22 mph, set TP_remaining = 0

    if ( stepper_furl_timer > 0 ) TP_remaining = (full_furl_time - stepper_furl_timer)/FURLCTL1_PER_SEC;  // if we're furled, show remaining furl time
    analog_channels[LINE_VOLTAGE].setInstantaneousValInt((TP_remaining*1024), false, false, false);  // TP hold time --> VL channel


    
    // FOR DEBUG: copy local vars to global vars for printing in loop()
    // We can't print them here because furlctl1() runs too fast!
    //print_ws_arrlen = ws_arrsize;
    //print_ws_arrsum = (ws_arrsum*MS2MPH)/1024.;
    //print_ws_avg = (ws_avg*MS2MPH)/1024.;
    //print_ws_max = (ws_max*MS2MPH)/1024.;
    //for (int ws = 22; ws < 41; ws++) {        // loop over only those wind speeds we want to know about
    //  print_hold_WSgte[ws] = hold_WSgte[ws];
    //}

  }  // END if (ave_count == FURLCTL1_PER_SEC)
  
  // ********** end COLLECT RPM AND WS MEASUREMENTS **********


  // ********** AUTO MODE **********
  // Allow Auto Mode only if we're: not in Manual Mode *AND* not initializing TP *AND* not in a shutdown state
  // NOTE: After coming out of a shutdown_state there is NO HISTORY to preset a safe TP based on predTP calculations.
  // So... could we move predTP code out of Auto Mode so that it also runs while in shutdown_state 1 or 2?
  //
  if ( !checkManualMode() && !initialize_TP && (shutdown_state == 0) ) {

    // Set FURL direction
    if ( (motor.currentPosition() == 0) && (motor_zero_flag == 0) ) { // if motor is at zero position, but flag is not set...
      motor_zero_flag = 1;  // set flag, disallowing this if() until unset below
      furlDir = -furlDir;   // toggle FURL direction
    }
    if ( (motor.currentPosition() != 0) && (motor_zero_flag == 1) ) { // if motor has moved away from zero position, but flag is still set...
      motor_zero_flag = 0;  // unset flag, disallowing this if() until set above
    }

    // Handle TAIL MOTOR SLIPPAGE which can occur over time, esp. in high winds.
    // Accumulate total angular motion of the tail and set a FURL flag when it exceeds some threshold.
    current_motor_position = abs( motor.currentPosition() );
    total_tail_motion += abs( (current_motor_position - last_motor_position) );  // usteps!
    last_motor_position = current_motor_position;
    slippage_furl = false;
    if ( total_tail_motion >= (3*2000*62) ) {  // if total accumulated tail motion >= 3 revs, where 1 rev = 360 deg = 2000*62 usteps...
      slippage_furl = true;                    //   set furl flag, total_tail_motion is reset below
    }

    // Now... use RPM and WS values to do some work:

    // FROZEN ANEMOMETER DETECTION
    // Here, we use single observations of WS and RPM
    if ( getChannelRMSInt(WINDSPEED) >= (1*1024) ) {     // if WS >= 1 m/s... 
      WS_nonzero++;                //   increment WS!=0 counter (anemometer is OK)
      WS_zero = 0;                 //   reset WS==0 counter
    } else {                       // otherwise...
      WS_zero++;                   //   increment WS==0 counter (anemometer *might* be frozen)
      WS_nonzero = 0;              //   reset WS!=0 counter
    }
    if ( getChannelRMSInt(RPM) >= (120*1024) ) {  // if RPM > 120...
      RPM_nonzero++;                //   increment RPM!=0 counter (turbine is spinning)
    } else {                        // otherwise...
      RPM_nonzero = 0;              //   reset RPM!=0 counter (turbine is not spinning)
    }
    // If anemometer_furl flag is false && we've seen 60s of RPM > 0 && we've seen 60s of WS==0, then anem is frozen!
    if ( !anemometer_furl &&
         (RPM_nonzero >= (60*FURLCTL1_PER_SEC)) && 
         (WS_zero >= (60*FURLCTL1_PER_SEC)) ) {
      anemometer_furl = true;
    }
    // If anememometer_furl flag is true && we've seen 60s of WS!=0 data, then anemometer is no longer frozen.
    // Besides a controller reset, this if() statement is the ONLY way to set anemometer_furl back to false.
    if ( anemometer_furl && 
         (WS_nonzero >= (60*FURLCTL1_PER_SEC)) ) {
      anemometer_furl = false;  // setting this flag false starts stepper_furl_timer, so keep ANEM full_furl_time short!
    }
    // Constrain counters so they can't go beyond 60s
    WS_zero = min( WS_zero, (60*FURLCTL1_PER_SEC) );
    WS_nonzero = min( WS_nonzero, (60*FURLCTL1_PER_SEC) );
    RPM_nonzero = min( RPM_nonzero, (60*FURLCTL1_PER_SEC) );
    // end FROZEN ANEMOMETER DETECTION


    // TAIL POSITIONING
    // To do this, we use *time-averaged* observations of WS and RPM.
    //
    // Even though it's time-averaged, rpm_avg is UPDATED at a rate of FURLCTL1_PER_SEC, so...
    // we multiply by FURLCTL1_PER_SEC to get dRPM/dt.
    if ( (rpm_avg >= (120*1024)) && (last_rpm_avg >= (120*1024)) ) {  // this prevents dRPM/dt furling at < 120 RPM
      dRPMdt = (rpm_avg - last_rpm_avg) * FURLCTL1_PER_SEC;           //   dRPM/dt = change in RPM per *second* (RPM/s, 1024x actual, int)
    } else {
      dRPMdt = -1;  // small (-) value is needed to allow unfurl at startup
    }
    // delta_t is an IMPORTANT parameter which sets the # seconds ahead to predict the RPM.
    // delta_t should probably be set just long enough to allow for rotor inertia (1-3 sec?), but not too long or we'll overfurl.
    // rpm_avg is a 0.5-sec averaged value centered 0.25 seconds in the *past*, so delta_t = 1.5 extrapolates 1.25 seconds ahead of that point.
    // Increasing delta_t increases sensitivity to RPM-based furling (since we're extrapolating further ahead in time, but acting *immediately* on the extrapolated value.
    // Instead of using a var, use *integer arithmetic* here to set delta_t, e.g., delta_t = 1.5 = 3 / 2
    predRPM = rpm_avg + (dRPMdt * 3);  // predicted RPM (1024x actual, int)
    //analog_channels[LINE_VOLTAGE].setInstantaneousValInt( predRPM, false, false, false );  // predRPM --> VL channel
    
    //print_dRPMdt = dRPMdt/1024;    // DEBUG, for printing in loop(), int
    //print_predRPM = predRPM/1024;  // DEBUG, for printing in loop(), int

    // NOTE: ws_avg is a 300-sec = 5-min val, so we can't use it! However...
    // ws_arr[ws_arrsize-1] is the last element of an array with 1-sec averaged wind speeds which is updated every SECOND (not at FURLCTL1_PER_SEC!)
    // ws_arr[ws_arrsize-2] is the penultimate element, 1-sec earlier.
    // These vals are exactly what we want to compute dWS/dt.
    if ( (ws_arr[ws_arrsize-1] >= (3*1024)) && (ws_arr[ws_arrsize-2] >= (3*1024)) ) {  // prevent dWS/dt furling at < 3 m/s
      dWSdt = ws_arr[ws_arrsize-1] - ws_arr[ws_arrsize-2];  // compute dWS/dt = (m/s per *second*, 1024x actual)
    } else {
      dWSdt = -1;  // small (-) value is needed to allow unfurl at startup
    }
    //print_dWSdt = (dWSdt*MS2MPH)/1024.;  // DEBUG, for printing in loop(), float!

    
    // CALCULATE PRE-SET TAIL POSITIONS.
    // We want to calculate pre-set tail positions (TP) for each WS threshold (WSgte[20]..WSgte[40]) such that turbine output power 
    // at each (partially furled) TP will = output power of the UNFURLED turbine at 20 mph.
    // ASSUMPTIONS:
    //   1. No furl is necessary at 20 mph (empirical observation), with 22 mph being the next higher WS to consider, AND
    //   2. power is proportional to cube of wind speed (FACT) --> v^3 dependence, AND
    //   3. power is proportional to rotor swept area (FACT) --> cos(furl_angle) dependence, A_ellipse/A_circle = PI*r*rcos(furl_angle)/PI*r*r = cos(furl_angle), AND
    //   4. power is proportional to angle of attack (AoA) which, in turn, is proportional to aerodynamic efficiency = L/D --> cos^2(furl_angle) dependence:
    //      lift L decreases and drag D increases as a function of furl angle, i.e.,
    //      L/D = L0*cos(furl_angle) / (D0/cos(furl_angle)) = (L0/D0)*cos^2(furl_angle) ***THIS NEEDS FURTHER RESEARCH***
    //     
    //   If #4 holds, then *relative* power = (ratio of WS)^3 is proportional to cos^3(furl angle):
    //     cos^3(furl_angle) = (WS@furl_angle=0 / WS@furl_angle)^3
    //     cos(furl_angle) = WS@furl_angle=0 / WS@furl_angle
    //     furl_angle = acos( WS@furl_angle=0 / WS@furl_angle )
    //
    // Amazingly enough, this works quite well in practice!
    //
    // For no furl at 20 mph:
    // TP @ 20 mph: acos( (20/20) ) = 0.0 deg
    // TP @ 22 mph: acos( (20/22) ) = 24.6 deg (L1)
    // TP @ 24 mph: acos( (20/24) ) = 33.6 deg (L2)
    // TP @ 26 mph: acos( (20/26) ) = 39.7 deg (L3)
    // TP @ 28 mph: acos( (20/28) ) = 44.4 deg (L4)
    // TP @ 30 mph: acos( (20/30) ) = 48.2 deg (L5)
    // TP @ 32 mph: acos( (20/32) ) = 51.3 deg (L6)
    // TP @ 34 mph: acos( (20/34) ) = 54.0 deg (L7)
    // TP @ 36 mph: acos( (20/36) ) = 56.3 deg (L8)
    // TP @ 38 mph: acos( (20/38) ) = 58.2 deg (L9)
    // TP @ 40 mph: acos( (20/40) ) = 60.0 deg (L10)

    // Next, we add a safety constraint based on very-near-future predicted turbine RPM (predRPM, calculated above) and combine that with
    // the WSgte[22]...WSgte[40] thresholds to set a predicted tail position (predTP).
    // 
    // rev0 BLADES:
    // 280-380rpm --> 1000-2000W, approximately linear
    // 340rpm-->1600W
    // 360rpm-->1800W
    // 380rpm-->2000W
    // 400rpm-->2200W
    // 420rpm-->2400W
    // 444rpm-->2600W (2021-12-16)
    // 463rpm-->2868W (2021-12-22)
    // 467rpm-->2816W (2022-04-01)
    // 472rpm-->2905W (2022-04-28), 231.5V, 12.55A
    //
    // rev1 BLADES:
    // 497 rpm --> 3037W, 300.7V, 10.10A (2022-07-18)
    // 
    // NOTE: We're only using WSgte[22]...WSgte[40] here. No need to calculate unused array elements.
    if ( (predRPM  < (380*1024)) && !WSgte[22] ) predTP = 0;                   // L0, && is necessary here
    if ( (predRPM >= (380*1024)) ||  WSgte[22] ) predTP = (24.6*2000*62)/360;  // L1, deg --> usteps
    if ( (predRPM >= (390*1024)) ||  WSgte[24] ) predTP = (33.6*2000*62)/360;  // L2
    if ( (predRPM >= (400*1024)) ||  WSgte[26] ) predTP = (39.7*2000*62)/360;  // L3
    if ( (predRPM >= (410*1024)) ||  WSgte[28] ) predTP = (44.4*2000*62)/360;  // L4
    if ( (predRPM >= (420*1024)) ||  WSgte[30] ) predTP = (48.2*2000*62)/360;  // L5
    if ( (predRPM >= (430*1024)) ||  WSgte[32] ) predTP = (51.3*2000*62)/360;  // L6
    if ( (predRPM >= (440*1024)) ||  WSgte[34] ) predTP = (54.0*2000*62)/360;  // L7
    if ( (predRPM >= (450*1024)) ||  WSgte[36] ) predTP = (56.3*2000*62)/360;  // L8
    if ( (predRPM >= (460*1024)) ||  WSgte[38] ) predTP = (58.2*2000*62)/360;  // L9
    if ( (predRPM >= (470*1024)) ||  WSgte[40] ) predTP = (60.0*2000*62)/360;  // L10
    
    // Save some vals in global print_ vars for slow printing in wwe.ino
    print_TP_timer = TP_timer;
    print_predTP = (predTP*360.)/(2000*62);


    // If stepper_furl_timer is non-zero, i.e., we're NOT furled already...
    if ( !stepper_furl_timer ) {
      
      // This if() needs to be BEFORE the next if() because the latter may move the tail, affecting currentPosition!
      if ( predTP <= abs(motor.currentPosition()) ) TP_timer++;

      // FURL (or increase FURL angle) IMMEDIATELY according to predicted TP.
      // Small overshoots can occur when, say, predTP = 40, then predTP decreases to 20 _just before_ actual TP reaches 20 (originally on its way to 40).
      // So... predTP_saved prevents these small overshoots because it resets after any unfurl event.
      if ( (predTP > abs(motor.currentPosition())) && (predTP > predTP_saved) ) {
        motorDPS = 20;                                                               // FURL @ 20 deg/s
        motorVelocity = (motorDPS*2000*62)/360;
        motor.maxVelocity( motorVelocity );
        if ( motor.currentPosition() > 0 ) motor.desiredPosition( predTP );          // FURL to (+) position
        if ( motor.currentPosition() < 0 ) motor.desiredPosition( -predTP );         // FURL to (-) position
        if ( motor.currentPosition() ==0 ) motor.desiredPosition( furlDir*predTP );  // FURL in direction determined by furlDir flag
        TP_timer = 0;                                                                // reset TP timer after FURLING tail
        predTP_saved = predTP;                                                       // save predTP to which we're furling
      }
        
      // Unfurl STEPWISE, holding each lower TP for duration of TP_timer until we're back to zero.
      // ASSUME: rotor inertia delays RPM response (to changing WS) by 'a few' sec. So, don't unfurl for AT LEAST that long.
      // UNFURL to predTP if: 
      //   TP_timer >= #seconds * FURLCTL1_PER_SEC                   --> 3-10 seconds
      //   *and* (rotor speed is decreasing *or* is < low threshold) --> 180 rpm
      //   *and* (wind speed is decreasing *or* is < low threshold)  --> 5 m/s
      //   *and* (rotor speed has decreased to a 'reasonable' RPM    --> 340 rpm
      //   *and* pred TP < actual TP
      //
      if ( (TP_timer >= (6*FURLCTL1_PER_SEC)) && 
           ( (dRPMdt < 0) || (rpm_avg < (180*1024)) ) && 
           ( (dWSdt < 0) || (ws_avg < (5*1024)) ) &&
           ( rpm_avg < (340*1024) ) &&
           ( predTP < abs(motor.currentPosition()) ) ) {
        motorDPS = 10;                                                          // UNFURL @ 10 deg/s (noisy resonance @ 5 deg/s)
        motorVelocity = (motorDPS*2000*62)/360;
        motor.maxVelocity(motorVelocity);
        if ( predTP < abs(motor.currentPosition()) ) {                          // UNFURL only if predTP is < current TP
          if ( motor.currentPosition() > 0 ) motor.desiredPosition( predTP );   // UNFURL to (+) position
          if ( motor.currentPosition() < 0 ) motor.desiredPosition( -predTP );  // UNFURL to (-) position
          TP_timer = 0;                                                         // reset TP timer after UNFURLING tail
          predTP_saved = 0;                                                     // reset predTP_saved after any unfurl
        }
      }
    }  // end if (!stepper_furl_timer)
    // end TAIL POSITIONING


    // If we've gone 24h without a full furl, set flag used by checkFurlConditions() and checkSCConditions()
    // Immediately below, this flag will set a non-zero furl_reason which, in turn, will rezero quiet_time
    if ( quiet_time > (24*60*60*FURLCTL1_PER_SEC) ) {
      exercise_furl = true;
    }

    // Check full furl conditions.
    furl_reason = checkFurlConditions();               // this resets furl_reason EVERY iteration
    furl_reason == 0 ? quiet_time++ : quiet_time = 0;  // if we don't have a furl condition, increment quiet_time, otherwise zero it
    
    // Evaluate furl reasons
    if ( furl_reason > 0 ) {
      furl_reason_saved = furl_reason;  // save furl_reason, used by getControllerState()
      total_tail_motion = 0;            // if we furl completely for ANY reason (including slippage!), reset slippage counter

      int furl_time_remaining = full_furl_time - stepper_furl_timer;  // local scope var

      // Set full_furl_time and, possibly, set flag for engaging the shorting contactor.
      // Set full_furl_time to nominal time for the furl reason OR to the *remaining* furl time (from an earlier full furl), WHICHEVER IS GREATER
      // Do NOT set engage_sc = false in this switch() statement! That is done immediably below in: if (stepper_furl_timer)...
      switch( furl_reason ) {
        case 1 ... 15:                                                       // VOLT and/or CURR and/or RPM and/or WIND over-threshold (bits 0-3)
          full_furl_time = max(600*FURLCTL1_PER_SEC, furl_time_remaining);   //   10-min furl, possibly longer if XWIND has been triggered
          break;
        case 16 ... 31:                                                      // EXERcise tail (bit 4)
          full_furl_time = max(30*FURLCTL1_PER_SEC, furl_time_remaining);    //   30-sec furl (needs to be > time required for furl to complete!)
          exercise_furl = false;                                             //   reset flag
          engage_sc = true;                                                  //   engage shorting contactor
          break;
        case 32 ... 63:                                                      // ANEMometer frozen (or no signal for any reason besides no wind) (bit 5)
          full_furl_time = max(1*FURLCTL1_PER_SEC, furl_time_remaining);     //   1-sec furl, anemometer_furl flag is reset in frozen anem detection code above
          engage_sc = true;                                                  //   engage shorting contactor
          break;
        case 64 ... 127:                                                     // SLIPpage reset of tail position (bit 6)
          full_furl_time = max(30*FURLCTL1_PER_SEC, furl_time_remaining);    //   30-sec furl, slippage_furl flag is reset in code above
          break;
        case 128 ... 255:                                                    // MORNingstar fault or wrong state (bit 7)
          full_furl_time = max(60*FURLCTL1_PER_SEC, furl_time_remaining);    //   1-min furl, morningstar_furl (int) is reset in code above
          engage_sc = true;                                                  //   engage shorting contactor
          break;
        case 256 ... 511:                                                    // eXtreme WIND conditions, WIND (8) and XWIND (256) will be triggered *simultaneously* = 264 (bit 8)
          full_furl_time = max(1800*FURLCTL1_PER_SEC, furl_time_remaining);  //   30-min furl
          engage_sc = true;                                                  //   engage shorting contactor
          break;
        case 512 ... 1023:                                                   // WX (weather) NWS Alert/Warning/Advisory furl (bit 9)
          full_furl_time = max(1*FURLCTL1_PER_SEC, furl_time_remaining);     //   1-sec furl, weather_furl flag is reset when alert expires by getNWSAPIData in webclient.ino
          engage_sc = true;                                                  //   engage shorting contactor
          break;
        default:                                                             // shouldn't happen, but if does...
          full_furl_time = max(3600*FURLCTL1_PER_SEC, furl_time_remaining);  //   60-min furl
          engage_sc = true;                                                  //   engage shorting contactor
      }
      stepper_furl_timer = 1;  // re/start furl timer. If we're already fully furled, another triggering event will reset full_furl_time.
    }  // END if ( furl_reason > 0 )

    // ***FURL***
    if ( stepper_furl_timer  ) {                    // if stepper_furl_timer is non-zero...
      stepper_furl_timer++;                         //   increment furl timer here (not further below due to exit outer if() condition below)
      
      if ( stepper_furl_timer > full_furl_time ) {  //   if we've exceeded full_furl_time...
        stepper_furl_timer = 0;                     //     exit the outer if() on the next iteration
        full_furl_time = 0;                         //     necessary because of the max() comparisons above
        furl_reason_saved = 0;                      //     reset saved furl reason, used by getControllerState()
        sc_reason_saved = 0;                        //     reset saved SC reason, used by getControllerState()
        engage_sc = false;                          //     ***disengage SC***
        TP_timer = 10000;                           //     re-initialize tail position timer
      }

      if ( abs(motor.currentPosition()) >= (RSlimit - 3444) ) {  // if we're close to RSlimit...
        motorDPS = 1;                                            //   furl SLOW to avoid overshoot (1 deg/s)
        motorVelocity = (motorDPS*2000*62)/360;                  //   usteps/s
        motor.maxVelocity(motorVelocity);
      } else {                                                   // otherwise...
        motorDPS = 20;                                           //   furl FAST (20 deg/s) 
        motorVelocity = (motorDPS*2000*62)/360;                  //   usteps/s
        motor.maxVelocity(motorVelocity);
      }
      // FURL until RS closes or FURLlimit is reached
      if (reed_switch_state != 1) {
        if (motor.currentPosition() >= 0) motor.desiredPosition(FURLlimit);
        if (motor.currentPosition() < 0) motor.desiredPosition(-FURLlimit);
      }
    }  // END if (stepper_furl_timer)
  }  
  // ********** end AUTO MODE **********
}  
// ********** end STEPPER MOTOR CONTROL - furlctl1() **********




// Check for Morningstar mppt600 and div60 controller faults. 
// Return sum of fault codes.
int checkMorningstarFaults() {
  if ( motor.isMotorOn() ) return 0;

  int f = mppt600_fault_i.valInt() + div60_fault.valInt();
  return f ;
}


// Check for Morningstar mppt600 and div60 controller alarms (these are warnings, not faults). 
// Return sum of alarm codes.
int checkMorningstarAlarms() {
  if ( motor.isMotorOn() ) return 0;
  
  int a =  mppt600_alarm_i.valInt() + div60_Alarm_LO.valInt();
  return a;
}


// Check Morningstar mppt600 and div60 operating state.
//   mppt600 and div60 cached data must be available, AND...
//   mppt600_mb_charge_state MUST be in MPPT(5) or DISC(2), AND...
//   div60_control_state MUST be in BULK(5) or PWM(6).
int checkMorningstarState() {
  // This is a workaround to avoid Morningstar furls due to bad Modbus long reads due to stepper motor driver noise.
  // Better shielding within the controller may allow this check to be eliminated.
  if ( motor.isMotorOn() ) return 0;
  
  int mppt600_state = mppt600_mb_charge_state.valInt();  // get mppt600 state
  int div60_state = div60_control_state.valInt();        // get div60 state
  
  if ( (mppt600.cachedDataOK() && (mppt600_state==5 || mppt600_state==2)) && 
       (div60.cachedDataOK()   && (div60_state==5   || div60_state==6)) ) {
    return 0;  // controllers are OK.
  } else {
    return 1;
  }
}


// This function checks for shorting contactor ON conditions.
//   Its only use is to set state sc_reason bits that get put into the bitfield returned by getControllerState().
//   It is NOT used to activate the contactor. For that, see 
int checkSCConditions() {
  int reason = 0;                                                    // local scope!
  if (threshold_checkers[SC_HIGH_V].check(false)) reason |= 1;       // if over-voltage (VOLT), set bit 0
  if (threshold_checkers[SC_HIGH_I].check(false)) reason |= 2;       // if over-current (CURR), set bit 1
  if (threshold_checkers[SC_HIGH_RPM].check(false)) reason |= 4;     // if over-speed (RPM), set bit 2
  if (threshold_checkers[SC_HIGH_TP].check(false)) reason |= 8;      // if over-furled (TAIL), set bit 3
  if ( tp_init_fail ) reason |= 16;                                  // if TP initialization fails (TPINIT), set bit 4
  if ( exercise_furl ) reason |= 32;                                 // if EXERcise tail flag, set bit 5
  if ( anemometer_furl ) reason |= 64;                               // if ANEMometer frozen flag, set bit 6
  if ( morningstar_furl ) reason |= 128;                             // if MORNingstar fault or wrong state flag, set bit 7
  if ( threshold_checkers[SC_HIGH_WS].check(false) ) reason |= 256;  // if eXtreme WIND flag, set bit 8
  if ( weather_furl && !parm_wx_override.intVal() ) reason |= 512;   // if WX (weather) flag and "WX Override?" == 0, set bit 9
  return(reason);                                                    // return 10 bits --> 2^10 --> possible values are 0-1023
}


// This function checks for full furl conditions.
// It returns a bitfield coding one or more furl reasons.
int checkFurlConditions() {
  int reason = 0;                                                     // local scope!
  if ( threshold_checkers[FURL_INIT_V].check(false) ) reason |= 1;    // if VOLTage threshold exceeded, set bit 0
  if ( threshold_checkers[FURL_INIT_I].check(false) ) reason |= 2;    // if CURRent threshold exceeded, set bit 1
  if ( threshold_checkers[FURL_INIT_RPM].check(false) ) reason |= 4;  // if RPM threshold exceeded, set bit 2
  if ( threshold_checkers[FURL_INIT_WS].check(false) ) reason |= 8;   // if WIND speed threshold exceeded, set bit 3
  if ( exercise_furl ) reason |= 16;                                  // if EXERcise tail flag, set bit 4
  if ( anemometer_furl ) reason |= 32;                                // if ANEMometer frozen flag, set bit 5
  if ( slippage_furl ) reason |= 64;                                  // if SLIPpage tail motor flag, set bit 6
  if ( morningstar_furl ) reason |= 128;                              // if MORNingstar fault or wrong state flag, set bit 7
  if ( threshold_checkers[SC_HIGH_WS].check(false) ) reason |= 256;   // if eXtreme WIND threshold exceeded, set bit 8
  if ( weather_furl && !parm_wx_override.intVal() ) reason |= 512;    // if WX (weather) flag and "WX Override?" == 0, set bit 9
  return(reason);                                                     // return 10 bits --> 2^10 --> possible values are 0-1023
}

int checkManualMode(){
  return(!digitalReadDirect(PIN_LOW_MANUAL_MODE));
}

boolean checkManualFurl(){
  return((!digitalReadDirect(PIN_LOW_FORCE_FURL)) && checkManualMode());
}

boolean checkManualUnfurl() {
  return((!digitalReadDirect(PIN_LOW_FORCE_UNFURL)) && checkManualMode());
}

void printCheckerChecks() {
  Serial << "V: " << threshold_checkers[FURL_INIT_V].check(true) << 
           " I: " << threshold_checkers[FURL_INIT_I].check(true) <<
           " RPM: " << threshold_checkers[FURL_INIT_RPM].check(true) <<
           " WS: " << threshold_checkers[FURL_INIT_WS].check(true) << "\n";  
}

int getFurlState() {
  return(furl_state);
}

int getFurlReason() {
  return(furl_reason_saved);
}


// This will be called to print debug info. ONLY CALL OUTSIDE OF TIMER LOOP!
void printFurlStatus(int msglvl){
  if (msglvl >= MSGLVL) {
    if (furl_state == UNFURL) {
      Serial << "furlctl: state = UNFURL";
    } else {
      Serial << "furlctl: state = FURL";
    }
    Serial << " furl motor = " << (furl_motor_on / FURLCTL_PER_SEC) 
           << " reason = " << furl_reason_saved
           << " unfurl_countdown/duration = " << (unfurl_countdown / FURLCTL_PER_SEC)
           << "/" << (furl_duration / FURLCTL_PER_SEC) << "\n";
    
    if (sc_shorted == 1) {     
      Serial << "furlctl: SC NOT energized (closed = SHORTED)\n";
      Serial << "furlctl: SC Failsafe Reason " << sc_failsafe_reason << "\n";
    } else {
      Serial << "furlctl: SC energized (OPEN)\n";
    }
    
    if (dump_load_duty_cycle > 0) {
      Serial << "furlctl: Dump load ON = " << dump_load_duty_cycle << "\n";
    } else {
      Serial << "furlctl: Dump load OFF\n";
    }
  }
}


// Check for conditions which guarantee a safe exercise of the shorting contactor.
boolean checkSCSafeConditions() {
  boolean rc = false;                                    // return code, local scope!
  rc = ( threshold_checkers[SC_LOW_V].check(false)       // true if V < low threshold V
      && threshold_checkers[SC_LOW_I].check(false)       // true if I < low threshold I
      && threshold_checkers[SC_LOW_RPM].check(false) );  // true if RPM < low threshold RPM
  return rc;
}


// Check for conditions which require an IMMEDIATE short.
// SC at high voltage, current or RPM carries some risk of alternator demagnetization, but may prevent even more severe damage.
int checkSCNowConditions() {
  int reason = 0;                                                 // local scope!
  if (threshold_checkers[SC_HIGH_V].check(false)) reason |= 1;    // if over-voltage (VOLT), set bit 0
  if (threshold_checkers[SC_HIGH_I].check(false)) reason |= 2;    // if over-current (CURR), set bit 1
  if (threshold_checkers[SC_HIGH_RPM].check(false)) reason |= 4;  // if over-speed (RPM), set bit 2
  if (threshold_checkers[SC_HIGH_TP].check(false)) reason |= 8;   // if over-furled (TAIL), set bit 3
  if (tp_init_fail) reason |= 16;                                 // if TP initialization fails (TPINIT), set bit 4
  return(reason);                                                 // return 5 bits, possible values are 0-31
}


// This function returns TRUE if the SC switch of the wired remote is ON
boolean checkManualSCShort() {
  return(!digitalReadDirect(PIN_LOW_SHORT_SC));  // PIN_LOW_SHORT_SC = pin 24 will be LOW (ground) when switch is ON
                                                 // so digitalReadDirect(0) == false, !digitalReadDirect(0) = true
}


// This function returns (much of) the current state of the controller as a 32-bit integer bitfield.
//   Discontinuities are the result of adding reasons for furl and SC as the system and code evolved.
//   NOTE: bitwise and shift operate on 32-bit ints
// Bits 0-2: sc_reason_saved, bits 7-9 --> MORN, XWIND, WX
// Bits 3-12: furl_reason_saved, bits 0-9 --> VOLT, CURR, RPM, WIND, EXER, ANEM, SLIP, MORN, XW, WX
// Bit 14: shorting contactor state: 0==unshorted, 1==shorted
// Bits 15-21: sc_reason_saved, bits 0-6 --> VOLT, CURR, RPM, TAIL, TPINIT, EXER, ANEM
// Bit 22: shutdown_state, bit 0: 0 --> SS==0, 1 --> SS==1
// Bit 23: reed switch state: 0==off, 1==on
// Bit 24: manual mode state: 0==auto, 1==manual
// Bit 25: stepper motor on/off
// Bit 26: stepper motor direction (+/-)
// Bit 27: shutdown_state, bit 1: 1 --> SS==2
// Bits 28-31: UNUSED

int getControllerState() {
  return( ((sc_reason_saved & 0x380) >> 7)    // select bits 7-9 (0011 1000 0000) of sc_reason_saved, shift *RIGHT* into return bits 0-2
        + ((furl_reason_saved & 0x3FF) << 3)  // select bits 0-9 (0011 1111 1111) of furl_reason_saved, shift LEFT into return bits 3-12
                                              // return bit 13 is UNUSED (save for one more furl reason?)
        + (sc_shorted << 14)                  // shift 1 bit LEFT into return bit 14
        + ((sc_reason_saved & 0x7F) << 15)    // select bits 0-6 (0111 1111) of sc_reason_saved, shift LEFT into return bits 15-21
        + ((shutdown_state & 1) << 22)        // select bit 0 of shutdown_state, shift LEFT into return bit 22
        + (!debounced_rs_state << 23)         // shift 1 bit LEFT into return bit 23
        + (checkManualMode() << 24)           // shift 1 bit LEFT into return bit 24
        + (motor.isMotorOn() << 25)           // shift 1 bit LEFT into return bit 25
        + (motor.motorDir() << 26)            // shift 1 bit LEFT into return bit 26
        + ((shutdown_state & 2) << 26)        // select bit 1 of shutdown_state, shift LEFT into return bit 27
                                              // return bits 28-31 are UNUSED
       );
}


// ********** DUMP LOAD CONTROL **********
// This function is called from adc.ino at 10000 Hz
// We use integer math throughout, no floating point!

// VOLTAGE-BASED DUMP LOAD CODE
// This function implements a PWM output to drive the dump load IGBT at 100 Hz.
// CRITICAL: Capacitance across the TS-MPPT-600V controller DC input is ABSOLUTELY REQUIRED to make this work, othewise...
//           as dump load switches on/off, we get Vdc spikes much > (Kb*RPM = Voc) and alternator makes horrible noise!
int manageDumpLoad() {
  static int dump_cyclenum = 0;                // dump cycle counter, must be static!
  static int Morningstar_timer = 0;            // function call iteration counter, must be static!
  static boolean MorningstarOK = false;        // controller ok flag
  static boolean unloaded = false;             // if true, this flag LATCHES dump load ON if obs'd vdc_drop is too low
  int dc_voltage = 0;                          // obs Vdc, 1024x actual
  int rpm_obs = 0;                             // obs RPM, ACTUAL value
  int vdc_drop = 0;                            // voltage drop, 1024x actual
  const int Kb = 1053;                         // alternator motor constant, 1024x actual: 1.0285 * 1024 = 1053

  // HVDL control parms
  //const int dump_load_resistance = 18 << 10;   // ohms, 1024x actual. This is FIXED (for now) even though it is a controller operating parm
  int dump_load_resistance = parm_hvdl_R.intVal() << 10;
  
  //const int dump_load_max_power = 2200 << 10;  // dump load maximum power (W), 1024x actual; rev0 blades 2200W
  //const int dump_load_max_power = 2200 << 10;  // dump load maximum power (W), 1024x actual; rev1 blades 2200W
  int dump_load_max_power = parm_hvdl_Pmax.intVal() << 10;

  //const int v_start = 120 << 10;               // dump start voltage, 1024x actual; rev0 and rev1 blades 120V
  int v_start = parm_hvdl_Vstart.intVal() << 10;
  
  //const int v_span = 110 << 10;                // dump span voltage, 1024x actual; rev0 blades 110V
  //const int v_span = 180 << 10;                // dump span voltage, 1024x actual; rev1 blades 180V
  int v_span = parm_hvdl_Vspan.intVal() << 10;

  const int max_duty = (dump_load_resistance * ((100*dump_load_max_power) / (v_start + v_span))) / (v_start + v_span);  // max dump load duty cycle
  
  // We need to know whether the TS-MPPT-600V is DISC'ed when manageDumpLoad() is first called by readADCs().
  // These initializations depend on mppt600_mb_charge_state having been read during Modbus initialization - see modbus.ino.
  // Usual states will be DISC==2 or MPPT==5.
  static int mppt600state = mppt600_mb_charge_state.valInt();
  static int last_mppt600state = mppt600_mb_charge_state.valInt();
  static boolean mppt600_disconnected = !(mppt600_mb_charge_state.valInt() - 2);  // if state is DISC==2, then !(2-2) = !0 = true

  static int mppt30state = mppt30_charge_state.valInt();
  static int last_mppt30state = mppt30_charge_state.valInt();
  static boolean mppt30_disconnected = !(mppt30_charge_state.valInt() - 2);

  static int mppt60state = mppt60_charge_state.valInt();
  static int last_mppt60state = mppt60_charge_state.valInt();  
  static boolean mppt60_disconnected = !(mppt60_charge_state.valInt() - 2);


  // Thereafter, as long as the Modbus is being read, this flag val will be the actual connection state of the wind controller.
  // Because mppt600_mb_charge_state is updated ONLY at 1 Hz intervals (the time between Modbus queries), we update the
  // mppt600_disconnected flag ONLY if the Modbus parm CHANGES. This prevents multiple writes to the TS-MPPT-600V controller below.
  mppt600state = mppt600_mb_charge_state.valInt();                                   // get the current state
  if ( mppt600state != last_mppt600state ) {                                         // if the state has changed...
    last_mppt600state = mppt600state;                                                //   save the state
    mppt600state == 2 ? mppt600_disconnected = true : mppt600_disconnected = false;  //   if the new state ==2 (DISC), set flag, otherwise unset it
  }
  mppt30state = mppt30_charge_state.valInt();
  if ( mppt30state != last_mppt30state ) {
    last_mppt30state = mppt30state;
    mppt30state == 2 ? mppt30_disconnected = true : mppt30_disconnected = false;
  }
  mppt60state = mppt60_charge_state.valInt();
  if ( mppt60state != last_mppt60state ) {
    last_mppt60state = mppt60state;
    mppt60state == 2 ? mppt60_disconnected = true : mppt60_disconnected = false;
  }

  // Dump Scenario 1 - this should occur very rarely:
  // Activate dump load if we detect an UNLOADED = OPEN CIRCUIT = FREEWHEELING condition.
  // This situation might arise, for example, with a blown DC input fuse between the TS-MPPT-600V and the WWE controller.
  // At 160rpm, we should see approx. 1.0A @ 145V, compared with Voc = Kb*rpm = 165V, which is a 20V drop.
  // *** A POWER ON/OFF RESET (or restart via the Arduino IDE) of the W2 controller is necessary to recover from this! ***
  dc_voltage = getChannelRMSInt(DC_VOLTAGE);                      // 1024x actual
  rpm_obs = getChannelRMSInt(RPM) >> 10;                          // shift RIGHT --> actual PRM
  vdc_drop = (Kb*rpm_obs) - dc_voltage;                           // compute voltage drop, diff is 1024x actual since Kb is 1024x actual
  ac_Pd.setInstantaneousValInt( vdc_drop, false, false, false );  // use Pd channel for voltage drop: 1024x actual, DON'T median, DON'T rectify, DON'T filter

  // If we're spinning fast enough to be at least lightly loaded AND voltage drop is < 10V, set dump load flag.
  if ( (rpm_obs > 160) &&  (vdc_drop < (10*1024)) ) {
    unloaded = true;
  } else {
    unloaded = false;
  }

  // Dump scenario 2:
  // Dump if TS-MPPT-600V (wind) *or* TS-60 (div) controller are in the wrong state OR report faults.
  // If faults resolve, we can stop dumping, but we need a few seconds delay for them to initialize. So...
  // Require controllers to be OK for 5 sec = 5000 msec = 50000 iter * 0.1 msec/iter before we stop dump load.
  if ( !unloaded && !mppt600_disconnected ) {                                    // if we're not unloaded AND wind controller is connected...
    if ( (checkMorningstarState() == 0) && (checkMorningstarFaults() == 0 ) ) {  //   if MS controllers are OK...
      Morningstar_timer++;                                                       //     don't increment counter inside min() - see Arduino min() documentation
      Morningstar_timer = min(Morningstar_timer, 50000);                         //     limit timer max value to 5 seconds
      if ( Morningstar_timer == 50000 ) {                                        //     if we reach 5 seconds... 
        dump_load_duty_cycle = 0;                                                //       reset because setting MorningstarOK=true excludes entry into dump_load_duty_cycle loop below!
        MorningstarOK = true;                                                    //       reset MS controller flag
        morningstar_furl = false;                                                //       reset MS furl flag
      }
    } else {                                                                     //   otherwise, controllers are not OK...
      Morningstar_timer = 0;                                                     //     rezero timer
      MorningstarOK = false;                                                     //     set not OK flag
      morningstar_furl = true;                                                   //     set furl flag --> a Morningstar fault needs investigation!
    }
  }

  // Dis/connect TS-MPPT-600V based on 'HVDL Active' controller parm val.
  if ( parm_hvdl_active.intVal() == 1 ) {  // if HVDL Active == 1 ...
    if (!mppt600_disconnected) {           //   if TS-MPPT-600V is connected...
      mppt600.writeSingleCoil(0x0002, 1);  //     disconnect it
      mppt600_disconnected = true;         //     set flag
      Serial << "furlctl: TS-MPPT-600V has been disconnected!\n";
    }
  }
  else {                                   // otherwise, HVDL Active == 0 ...
    if (mppt600_disconnected) {            //   if TS-MPPT-600V is disconnected...
      mppt600.writeSingleCoil(0x0002, 0);  //     connect it
      mppt600_disconnected = false;        //     unset flag
      Serial << "furlctl: TS-MPPT-600V has been reconnected.\n";
    }
  }
  // Same for TS-MPPT-30 (PV1)
  if ( parm_PV1_disc.intVal() == 1 ) {
    if (!mppt30_disconnected) {
      mppt30.writeSingleCoil(0x0002, 1);
      mppt30_disconnected = true;
      Serial << "furlctl: TS-MPPT-30 has been disconnected!\n";
    }
  }
  else {
    if (mppt30_disconnected) {
      mppt30.writeSingleCoil(0x0002, 0);
      mppt30_disconnected = false;
      Serial << "furlctl: TS-MPPT-30 has been reconnected.\n";
    }
  }
  // Same for TS-MPPT-60 (PV2)
  if ( parm_PV2_disc.intVal() == 1 ) {
    if (!mppt60_disconnected) {
      mppt60.writeSingleCoil(0x0002, 1);
      mppt60_disconnected = true;
      Serial << "furlctl: TS-MPPT-60 has been disconnected!\n";
    }
  }
  else {
    if (mppt60_disconnected) {
      mppt60.writeSingleCoil(0x0002, 0);
      mppt60_disconnected = false;
      Serial << "furlctl: TS-MPPT-60 has been reconnected.\n";
    }
  }

  // Activate dump load if: 
  //   controllers are not ready OR 
  //   wind controller is disconnected OR 
  //   an unloaded condition has been detected
  if ( !MorningstarOK || mppt600_disconnected || unloaded ) {
  
    // Update dump_load_duty_cycle every 100th function iteration.
    if ( dump_cyclenum >= 100 ) {  // dump load runs at 10,000 Hz / 100 = 100 Hz
      dump_cyclenum = 0;
          
      // If vstart + v_span > 120+110 = 230V, we want MAX and CONSTANT power = 2200W = max_duty * (V^2/R).
      //   --> max_duty = 100*2200*18/(230^2) = 75%, initialized above.
      // BE CAREFUL with integer math to avoid overflow at 2^31 - 1:
      //   --> max_duty = {(18*1024)*[(100*2200*1024)/(230*1024)]}/(230*1024) = 74.9%
      
      if ( dc_voltage >= v_start ) {              // if dc_voltage is ABOVE v_start (both are 1024x actual)...
        if ( dc_voltage > (v_start + v_span) ) {  //   if dc_voltage has exceeded (vstart + v_span)...
          // Above 230V, duty cycle DECREASES as voltage rises to MAINTAIN power at dump_load_max_power = 2200W
          dump_load_duty_cycle = (dump_load_resistance * ((100*dump_load_max_power) / dc_voltage)) / dc_voltage;  // GLOBAL var
        } else {                                  //   otherwise, dc_voltage is BETWEEN v_start and (vstart + v_span)... 
          // CREATE A SMOOTH LOAD CURVE:
          // Duty cycle increases linearly with V as power increases with V^2, so P(V) = duty(V) * (V^2)/R
          // ***This works amazingly well. So much so that this equation is used to calculate the P(V) load curve programmed into the TS-MPPT-600V.***
          //    Because the load curves are identical, switching between the TS-MPPT-600V and the HVDL should be seamless.
          dump_load_duty_cycle = (max_duty * (dc_voltage - v_start)) / v_span;
        }
      } else {                                    // otherwise, dc_voltage is BELOW v_start...
        dump_load_duty_cycle = 0;                 //   so no load yet
      }
    }  // END update dump_load_duty_cycle

    // Turn dump load ON for dump_load_duty_cycle iterations (where 1 iteration = 0.1 msec) out of every 10 msec
    if ( dump_cyclenum < dump_load_duty_cycle ) {
      digitalWriteDirect(DUMP_IGBT_DRV_PIN, DUMP_LOAD_ON);
      digitalWriteDirect(DUMP_LOAD_ACTIVE_LED_PIN, LOW);
    } else {
      digitalWriteDirect(DUMP_IGBT_DRV_PIN, DUMP_LOAD_OFF);
      digitalWriteDirect(DUMP_LOAD_ACTIVE_LED_PIN, HIGH);
    }
  
    dump_cyclenum++;
    
  }  // END activate dump load if()
  
}  // end manageDumpLoad()
