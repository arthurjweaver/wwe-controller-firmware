// ---------- adc.ino ----------
// ***THIS IS THE MAIN MEASUREMENT MODULE***
// First, several classes are defined:
//   class filter3DBInt
//   class AnalogChannelBase
//   class AnalogChannel: public AnalogChannelBase
//   class AnalogDiffChannel: public AnalogChannelBase
//   class RPMChannel: public AnalogChannelBase
//   class RPM2Channel: public AnalogChannelBase
//   class PLLChannel: public AnalogChannelBase
//
// Thereafter, we have the following functions:
//   void initADCOffsets()
//   void readADCs() --> called at 10000 Hz in wwe.ino, reads individual analog channels at 1000 Hz <--
//   void printAnalogChannels()
//   float *getChannelWaveformData(int channel)
//   char* getChannelName(int channel)
//   void printChannelsRMS()
//   int getChannelRMSInt(int channel)
//   void setTestValue(int index, float value)
//   void startCollectingWaveforms()
//   boolean checkCollectingWaveforms()
//   void debounceRS()

// SIGNAL FILTERING NOTES:
// RC low-pass filter approximation using a digital filter:
// w_out = alpha * w_in + (1 - alpha) * w_out, where w = angular freq.
//
// In the definition below, "signals" are frequency values - see https://en.wikipedia.org/wiki/Low-pass_filter
// "A low-pass filter is a filter that passes signals with a frequency lower than a selected cutoff frequency 
// and attenuates signals with frequencies higher than the cutoff frequency." 
//
// Cutoff frequency is defined as the *frequency* at which signal power is reduced by half, or approx. -3 dB = 10^(-3/10) = 0.5012
// If done at a sampling interval of h (sec), cutoff frequency = f_3dB = alpha / (2*pi * h * (1 - alpha))
// See https://en.wikipedia.org/wiki/Cutoff_frequency
// See https://en.wikipedia.org/wiki/Decibel
//
// More useful, if we want a particular f_3db, we can rearrange the equation above to get:
//   alpha = 1 / ( 1 + (1/(f_3db * 2*pi * h)) )
// -3dB @ 10 Hz cutoff filter, 1000 Hz sampling rate: alpha = 1 / ( 1 + (1/(10 * 2*pi * 0.001)) ) = 0.059 <-- used in AnalogChannelBase class
// -3dB @ 60 Hz cutoff filter, 1000 Hz sampling rate: alpha = 1 / ( 1 + (1/(60 * 2*pi * 0.001)) ) = 0.274 <-- used in AnalogDiffChannel class
// -3dB @ 60 Hz cutoff filter, 10000 Hz sampling rate: alpha = 1 / ( 1 + (1/(60 * 2*pi * 0.0001)) ) = 0.036 <-- used in RPMChannel class
// -3dB @ 200 Hz cutoff filter, 10000 Hz sampling rate: alpha = 1 / ( 1 + (1/(200 * 2*pi * 0.0001)) ) = 0.112
// -3dB @ 500 Hz cutoff filter, 10000 Hz sampling rate: alpha = 1 / ( 1 + (1/(500 * 2*pi * 0.0001)) ) = 0.239
// -3dB @ 1000 Hz cutoff filter, 10000 Hz sampling rate: alpha = 1 / ( 1 + (1/(1000 * 2*pi * 0.0001)) ) = 0.386
//
// What cutoff filter do we apply?
// The maximum rotor speed (RPM) that we would like to measure is approx. 600 rev/min * 1 min/60s = 10 rev/s = 10 Hz.
// But, there are 6 pole pairs, so the maximum sensed frequency is 60 Hz. We want to filter out any noise with a frequency
// higher than 60 Hz and "pass" frequencies lower than 60 Hz. So, we should apply a 60 Hz, 3dB low-pass filter.
//
// "The moving average operation used in fields such as finance is a particular kind of low-pass filter." 
// See https://en.wikipedia.org/wiki/Low-pass_filter
// See https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
// So, the EMA is effectively an RC filter which gives an average value to be used for amplitude measurement. 
// The scaled analog value is also saved in a waveform array for diagnostics.


// BEGIN function declarations. When are these needed?
void furlctl();
void furlctl1();
void writeStatus1();
void digitalWriteDirect();
int getControllerState();
int manageDumpLoad();
// END function declarations


// BEGIN variable declarations

// Define alphas for various low-pass filters:
const float AVG_ALPHA = 0.0591;         // This alpha is used in the AnalogChannelBase class and applied at 1000 Hz. (cw had this at 0.06)
                                        // ( This alpha is applied in the absence of a do_alpha arg to setInstantaneousValInt() OR
                                        // if the do_alpha arg to setInstantaneousValInt() is true ) AND...
                                        // AVG_ALPHA has not been overriden it using the setAlpha() method - see initADCOffsets()
                                        
const float DIFFCHANNEL_ALPHA = 0.274;  // This alpha is used by the *original* RPM channel (ac_freq1) and applied at 1000 Hz. (cw had this at 0.2)
                                        // In the AnalogDiffChannel class, this alpha filters out high frequency noise in the RPM *calculation*, then...
                                        // the resulting RPM *measurement* is pushed to setInstantaneousValInt() where it is FILTERED AGAIN, but not with 
                                        // the default AVG_ALPHA. Instead, there is a call to ac_freq1.setAlpha() in initADCOffsets().
                                        
const float FREQCHANNEL_ALPHA = 0.036;  // This alpha is used by the *new* RPM channel (ac_freq) in class RPMChannel and applied at 10000 Hz! (cw had this at 0.1)
                                        // In the RPMChannel class, this alpha filters out high frequency noise in the RPM *calculation*, then...
                                        // the resulting RPM *measurement* is pushed to setInstantaneousValInt() where it is FILTERED AGAIN 
                                        // since do_alpha is explicitly set true. As above, there is a call to ac_freq.setAlpha() in initADCOffsets().

// Attach names to each numbered channel
const int L1_VOLTAGE = 0;
const int L2_VOLTAGE = 1;
const int L3_VOLTAGE = 2;
const int DC_VOLTAGE = 3;
const int LINE_VOLTAGE = 4;
const int L1_CURRENT = 5;
const int L2_CURRENT = 6;
const int L3_CURRENT = 7;
const int DC_CURRENT = 8;
const int L1L2_VOLTAGE = 9;
const int L2L3_VOLTAGE = 10;
const int L3L1_VOLTAGE = 11;
const int RPM = 12;
const int WINDSPEED = 13;
const int DUMP_LOAD_P = 14;
const int RECT_TEMP = 15;
const int AMBIENT_TEMP = 16;
const int CONTROLLER_TEMP = 17;
const int TAIL_POSITION = 18;
const int WATT_HOURS = 19;
const int HVDL_DUTY_CYCLE = 20;
const int STATE = 21;             // controller state MUST be the last channel
const int NUM_ADC_CHANNELS = 22;  // total # includes STATE //

// Define voltage scale factors:
// Vin goes through a 1M/4.7K divider, and we have 4095 counts per 3.3V, so...
// voltage scale factor = 3.3/4095 V/count * (1000000+4700)/4700 = 0.1723 V/count
const float VOLTAGE_SCALE_FACTOR = 0.1723;      // 0.1723 V/count * 4095 counts = 706 V full-scale 

const float LINE_VOLTAGE_SCALE_FACTOR = 0.251;  // This is OBSOLETE. We're not measuring LV any more, but using the channel for other data.
                                                // Let's retain it though... just in case.

// Define current scale factor:
// To convert ADC Voltage --> Amps:
// The CT output voltage is divided down to prevent any possibility of it going above 3.3V which could damage the analog input
// of the Arduino. So, we have a voltage divider = 6.81/(13.3+6.81) = 0.6614 which scales 5V down to ~3.3V: 0.6614 * 5.00V = 3.31V.
// Analog inputs have 4095/3.3 counts/V, and CASR-15 CT sensitivity of 0.04167 V/A
// So, we have an analog input current scale factor = (6.81+13.3)/13.3 * (3.3/4095) V/count * (1/0.04167) A/V = 0.02924 A/count.
// Hardware measured currents are (-), so we negate the scale factor to make them (+).
const float CURRENT_SCALE_FACTOR = -0.02924;

// Define current offset:
// The "zero current sense" voltage output for the current sensor is approximately 2.5V, matching the voltage reference of the CT.
// The current sense outputs are shifted down through a voltage divider, so zero point should be at:
// 2.5V * 13.3/(6.81+13.3) * 4095/3.3 counts/V = 2052 counts
const int CURRENT_OFFSET = -2052;  // offset is (-)
float actual_current_offset = 0;   // var for real-time calculation, see readADCs() below.

// This const sets the array size for waveform[] in class AnalogChannelBase.
// In readADCs(), waveform_ptr is incremented when adc_index==9, so at a rate of 1000 Hz (1 msec intervals).
// This could be reduced to 0.1 msec by moving waveform_ptr out of the adc_index regime so it increments *every* readADCs() iteration.
// MAIN_SAMPLE_PERIOD_MILLIS would also need to be changed - see web.ino
//
// Calling sequence: HTTP request to the controller webserver, e.g., 192.168.1.40/wave.json 
// --> webserver.ino: waveCmd() --> adc.ino: startCollectingWaveforms()+checkCollectingWaveforms() --> webserver.ino: printChannelWaveJSON() 
// --> adc.ino: getChannelWaveformData() --> acs[channel]->getWaveformData() returns waveform[]
const int SAMPLES_IN_WAVEFORM = 256;  // originally 64; 128 ok, 256 ok, 512 ok, 1024 fails (compiler error)

// vars for waveform collection
boolean collect_waveforms = false;  // startCollectingWaveforms() sets this ==true
int waveform_ptr = 0;               // current index of waveform[]

// test data
float test_values[20];  // array used in place of real data if we're in test mode (see wwe.ino: ifdef USE_TEST_VALS)

// END variable declarations


// BEGIN class definitions

// This class implements a low-pass filter using integer arithmetic.
class filter3DBInt {
  protected:
    int alpha_int;
    int one_minus_alpha_int;
    int filtered_val_int;
    boolean do_abs;

  public:
    // alpha_int is 1024x actual alpha, which is between 0 and 1.
    filter3DBInt(int alpha_int, boolean do_abs): 
                 alpha_int(alpha_int), do_abs(do_abs) {
      one_minus_alpha_int = 1024 - alpha_int;
    }
    
    int doFilter(int val_int) {  // val_int is (usually) 1024x actual
      if (do_abs) {
        filtered_val_int = ((alpha_int * abs(val_int)) + (one_minus_alpha_int * filtered_val_int)) >> 10;
      } else {
        filtered_val_int = ((alpha_int * val_int) + (one_minus_alpha_int * filtered_val_int)) >> 10;
      }
      return(filtered_val_int);
    }
};  // END class filter3DBInt


// class AnalogChannelBase is a BASE class for:
//   9 raw data channels: V1, V2, V3, VDC, LV, I1, I2, I3, IDC (see class AnalogChannel)
//   3 voltage diff channels: L12, L23, L31 (see class AnalogDiffChannel)
//   6 other analog channels: WS, Pd,  Tr, Ta, Tc, TP
//   various RPM channels (see class RPMChannel, class RPM2Channel, class PLLChannel)
// The controller STATE channel is handled separately.
class AnalogChannelBase {
  protected:
    char* channel_name;
    int dc_offset;
    int avg_val_int;
    int alpha_int;
    int med_val_int;
    int instantaneous_val_int;
    float waveform[SAMPLES_IN_WAVEFORM];
    int valsptr_int = 0;
    int vals_int[3] = {0, 0, 0};

  public:
    AnalogChannelBase(char* channel_name): 
      channel_name(channel_name), dc_offset(0), avg_val_int(0) {
        
      alpha_int = (int)(1024.0 * AVG_ALPHA);  // The default AVG_ALPHA is used for filtering if:
                                              //   do_alpha arg to setInstantaneousValInt() is true, AND
                                              //   setAlpha() method has not been called - as it is initADCOffsets()!
    }

    // This function sets the instantaneous value of a channel.
    // With a single arg, all the filters are applied. With boolean args:
    //   do_median==true --> apply a 3-value median filter to reduce transient spikes
    //   do_abs==true --> rectify (take absolute value of) the val
    //   do_alpha==true --> apply a low-pass filter; alpha values (normally 0 to 1) are 1024x actual, i.e., 0-1 --> 0-1024.
    void setInstantaneousValInt(int val_int) {
      setInstantaneousValInt(val_int, true, true, true);
    }
    void setInstantaneousValInt(int val_int, boolean do_median, boolean do_abs, boolean do_alpha) {
      if (do_median) {
        med_val_int = getMedianValInt(val_int);
      } else {
        med_val_int = val_int;
      }
      
      if (do_abs) med_val_int = abs(med_val_int);
      
      if (do_alpha) {
        avg_val_int = ((alpha_int * med_val_int) + ((int)(1024 - alpha_int) * (avg_val_int))) >> 10;  // result is >> 10 because alpha_int has been scaled by 1024x
      } else {
        avg_val_int = med_val_int;
      }

      instantaneous_val_int = med_val_int;  // NOT TIME-AVERAGED!
    }

    float* getWaveformData() {
      return(waveform);
    }
    
    char* getName() {
      return( channel_name );
    }

    // This function returns a channel val which has been (possibly) median-filtered, (possibly) rectified, but NOT TIME-AVERAGED
    int getInstantaneousValInt() {
      return( instantaneous_val_int );
    }

    int getInstantaneousValIntPos() {
      return( instantaneous_val_int > 0 ? instantaneous_val_int : 0 );
    }

    // This function defines an offset to be applied to a raw int from the ADC.
    void setDCOffset(int offset) {
      dc_offset = offset;
    }
    
    // This function returns a channel val which has been (possibly) median-filtered, (possibly) rectified, AND (possibly) time-averaged.
    int getAvgValInt() {
      return( avg_val_int );
    }

    // This function defines a low-pass filter constant used in averaging.
    void setAlpha(float a) {
      alpha_int = (int)(a * 1024.0);
    }
    
    // This function returns a median val for the channel.
    int getMedianValInt(int val_int) {
      vals_int[valsptr_int++] = val_int;
      if (valsptr_int >= 3) valsptr_int = 0;
      if ( (vals_int[0] > vals_int[1]) && (vals_int[2] > vals_int[1]) ) {
        if (vals_int[0] <= vals_int[2] ) {
          return(vals_int[0]);
        } else {
          return(vals_int[2]);
        }
      } else {
        return(vals_int[1]);
      }
    }
};  // END class AnalogChannelBase


// class AnalogChannel EXTENDS the AnalogChannelBase class.
// This is a class for 9 raw data analog channels: 3 AC voltages, DC voltage, Line Voltage, 3 AC currents, and DC current.
// All analog inputs come into a 12-bit A/D converter, giving (theoretically) 12 bits (4096 points) of resolution. 
// Each analog channel is processed by AnalogChannel class methods.
class AnalogChannel: public AnalogChannelBase {
  private:
    int channel_num;
    float scale;
    int scale_int;
    int raw_val;

  public:
    AnalogChannel(int num, float scale, char* channel_name): 
                  channel_num(num), scale(scale), AnalogChannelBase(channel_name) {
                    
      scale_int = (int)(scale * 1024);
    }

    // This function reads an analog channel.
    // analogRead() returns an int to which a DC offset is applied to get the channel raw value.
    // Then, a 1024x scale factor, a median filter, NO rectification, and a low-pass filter are applied
    // and the result is saved with setInstantaneousValInt().
    void read() {
      raw_val = dc_offset + analogRead(channel_num);                    // dc_offset is non-zero for *current* channels - see initADCOffsets() below
      setInstantaneousValInt( (scale_int*raw_val), true, false, true);  // multiply channel value by scale_int (= 1024*scale), do median, DON'T rectify, do low-pass
      waveform[waveform_ptr] = getInstantaneousValInt()/1024.;          // save actual val as float in waveform[] - see AnalogChannelBase class
                                                                        // waveform_ptr increments when startCollectingWaveforms() is called - see below
    }
    
    // This function gets the hardware channel number, used in analogRead().
    int getChannelNum() {
      return(channel_num);
    }

    // This function returns the raw value.
    int getRawVal() {
      return(raw_val);
    }
};  // END class AnalogChannel


// We define ac_freq1 here (before referencing it in class AnalogDiffChannel) instead of later on with the other
// AnalogChannelBase channels; otherwise, we get a compiler error: 'ac_freq1' was not declared in this scope.
// ac_freq1 is the *original* frequency channel which has been *superseded* by ac_freq (class RPMChannel)- see below.
// We retain ac_freq1 for debugging and comparison with new freq determination methods.
AnalogChannelBase ac_freq1("RPM");


// class AnalogDiffChannel EXTENDS the AnalogChannelBase class.
// Phase-to-phase voltage waveforms are reconstructed by taking the differences between AC voltage waveforms L1, L2 and L3. 
// The calcDiff() method takes the difference of 2 analog channels, and sends it to setInstantaneousValInt(). 
// If calcDiff() arg == true, the diff'ed value is also sent through another low-pass filter with a cutoff frequency of
// about 100 Hz which smoothes the waveform for frequency calculation.
class AnalogDiffChannel: public AnalogChannelBase {
  private:
    AnalogChannel *chan1;
    AnalogChannel *chan2;
    int diff_alpha_int;
    int crossing_reference_int;
    int thediff_int;
    int filtered_val_int;
    int last_filtered_val_int;
    int filtered_val1_int;
    int last_filtered_val1_int;
    unsigned long cycle_start_time;
    float freq;
    unsigned long period;

  public:
    AnalogDiffChannel(AnalogChannel* c1, AnalogChannel* c2, char* channel_name): 
                      chan1(c1), chan2(c2), AnalogChannelBase(channel_name) {
      
      diff_alpha_int = (int)(DIFFCHANNEL_ALPHA * 1024.0);
      crossing_reference_int = 0;
    }

    // This function calculates the difference between 2 analog voltage channels and saves it in the ac_freq1 channel.
    // This method of frequency determination has been superseded by ac_freq in class RPMChannel.
    // If check_zero_crossing == true, then the diff is used for frequency calculation 
    // This function is called below in readADCs() when adc_index==9 as: 
    //   l1l2_diff.CalcDiff(true);  // <-- now set to false!
    //   l2l3_diff.CalcDiff(false);
    //   l3l1_diff.CalcDiff(false);
    void CalcDiff(boolean check_zero_crossing) {
      //digitalWriteDirect(STATUS1_PIN_1, HIGH);  // logic analyzer

      // getInstantaneousValInt() returns instantaneous_val_int which is: 
      // 1024x actual, scaled value read from the ADC, (possibly) median-filtered, (possibly) rectified, NOT time-averaged
      thediff_int = chan1->getInstantaneousValInt() - chan2->getInstantaneousValInt();
      setInstantaneousValInt(thediff_int, true, false, true);   // do median, DON'T rectify, do low-pass
      waveform[waveform_ptr] = getInstantaneousValInt()/1024.;  // save actual val as float in waveform[] - see AnalogChannelBase class
                                                                // waveform_ptr increments when startCollectingWaveforms() is called - see below

      if (check_zero_crossing) {
        //digitalWriteDirect(STATUS1_PIN_2, HIGH);  // logic analyzer

        // Apply DIFFCHANNEL_ALPHA *twice* to reduce noise for frequency calculation.
        last_filtered_val_int = filtered_val_int;
        filtered_val_int = ((diff_alpha_int * thediff_int) + ((1024 - diff_alpha_int) * last_filtered_val_int)) >> 10; 
        last_filtered_val1_int = filtered_val1_int;
        filtered_val1_int = ((diff_alpha_int * filtered_val_int) + ((1024 - diff_alpha_int) * last_filtered_val1_int)) >> 10;

        // Check for a positive zero crossing, where the crossing point is given by crossing_reference_int (default = 0)
        if ( (last_filtered_val1_int < crossing_reference_int) && (filtered_val1_int >= crossing_reference_int) ) {
          //digitalWriteDirect(STATUS1_PIN_4, HIGH);  // logic analyzer

          unsigned long now = micros();                   // # usec since the Arduino board began running the current program
          unsigned long period = now - cycle_start_time;  // # usec since *last* zero crossing
          cycle_start_time = now;                         // save cycle_start_time for the *current* zero crossing
          
          // Calculate frequency only if period is 'long enough', i.e., reject short periods (high frequencies) assuming they result from noise.
          // If we assume max rotor freq is 833 rpm = 13.9 rev/s * 6 cycles/rev = 83.3 Hz, 
          // then min period = 1/83.3 = 0.012 s = 12.0 msec = 12000 usec
          if (period > 12000) freq = 1024000000 / period;  // 1024 * 1000000 usec/s * 1 cycle/ period usec = 1024 * cycles/s = 1024 * Hz 
        }
        
        // 9-22-15 (cw): Our measurement system is relatively noisy. 
        // At low voltages, zero crossing detection shows false detects, so frequency/period is unreliable, also RMS voltage.
        // We need to act as if they are all zero for the purposes of turbine control. Let's revisit this!
        if ( abs(getAvgValInt()) < 30000 ) freq = 0;  // reject result if voltage diff < 30000/1024 = 29 V. 
                                                      // Added abs() so unrectified diffs can be saved as (+) or (-).

        // ac_freq1 is the *original* frequency channel which is *superseded* by ac_freq - see class RPMChannel below.
        ac_freq1.setInstantaneousValInt( (freq*60/parm_alt_poles.intVal()), true, true, true );  // do median, do rectify, do low-pass
      } // end if(check_zero_crossing);
    }

    void setCrossingReference(float ref) {
      crossing_reference_int = (int)(ref * 1024.0);
    }
};  // END class AnalogDiffChannel


// class RPMChannel EXTENDS the AnalogChannelBase class.
// This class is used ONLY for channel ac_freq which is instantiated below as: RPMChannel ac_freq(0, 1, "RPM", 200);
// The sample_and_test() function is called below at the full ADC rate (= 10000 Hz) and does the following:
//   1. calculates the diff between two voltage channels, 
//   2. filters the result through two 3db filters, then
//   3. uses hysteresis to further reject noise and create a square (0 or 1) wave,
//   4. computes freq from the period between square wave leading edges.
class RPMChannel: public AnalogChannelBase {
  private:
    int channel_num1;
    int channel_num2;
    int hysteresis;
    int minus_hysteresis;
    filter3DBInt filt1;  // separate instances are needed because of recursive calculation
    filter3DBInt filt2;  
    int rpm;

  public:
    RPMChannel(int num1, int num2, char* channel_name, int hyst): 
               channel_num1(num1), channel_num2(num2), 
               AnalogChannelBase(channel_name), 
               hysteresis(hyst/2), minus_hysteresis(-1 * hyst / 2),
               filt1( (int)(FREQCHANNEL_ALPHA * 1024.0), false ),  // args are: alpha*1024, take abs(int) - see class filter3DBInt
               filt2( (int)(FREQCHANNEL_ALPHA * 1024.0), false ) {
    }

    void sample_and_test() {
      static int squared_state = 0;
      static int last_squared_state = 0;
      static int last_filtered_val = 0;
      static int period_count = 0;
      int period = 0;
      int freq = 0;
      
      int raw_val1 = analogRead(channel_num1);  // raw val is 0-4095
      int raw_val2 = analogRead(channel_num2);
      int thediff = raw_val1 - raw_val2;                        // goes (+) and (-)
      int filtered_val1 = filt1.doFilter(thediff << 10);        // apply FREQCHANNEL_ALPHA to thediff*1024 --> filtered_val1, 1024x actual
      int filtered_val2 = filt2.doFilter(filtered_val1) >> 10;  // apply FREQCHANNEL_ALPHA to filtered_val1 --> filtered_val2, NOT 1024x actual
      
      // Use hysteresis to create a square wave (either 1 or 0)
      last_squared_state = squared_state;   
      if (squared_state == 0) {
        if (filtered_val2 > hysteresis) squared_state = 1;        // if we're definitely (+), hyst/2 = 100 * VOLTAGE_SCALE_FACTOR = 17.23V
      } else {
        if (filtered_val2 < minus_hysteresis) squared_state = 0;  // if we're definitely (-), -hyst/2 = -100 * VOLTAGE_SCALE_FACTOR = -17.23V
      }

      // Some relevant rotor speeds:
      // 2000 rpm = 33.3 rev/s * 6 cycles/rev = 200 Hz = 200 cycles/s --> period = 0.005 s/cycle --> *10000 iter/s = 50 iter/cycle = period_count (HIGH freq cutoff)
      //  600 rpm = 10 rev/s * 6 cycles/rev = 60 Hz = 60 cycles/s --> period = 0.0167 s/cycle --> *10000 iter/s = 167 iter/cycle (max rotor speed)
      //  300 rpm = 5 rev/s * 6 cycles/rev = 30 Hz = 30 cycles/s --> period = 0.0333 s/cycle --> *10000 iter/s = 333 iter/cycle (typ. rotor speed)
      //   20 rpm = 0.333 rev/s * 6 cycles/rev = 2 Hz = 2 cycles/s --> period = 0.500 s/cycle --> *10000 iter/s = 5000 iter/cycle (LOW freq cutoff)
      //   10 rpm = 0.167 rev/s * 6 cycles/rev = 1 Hz = 1 cycles/s --> period = 1.000 s/cycle --> *10000 iter/s = 10000 iter/cycle (LOW freq cutoff)
      
      if ( (squared_state - last_squared_state) == 1 ) {  // if we have a positive edge, i.e., squared_state == 1 and last_squared_state == 0
        period = period_count;                            //   save measured period (units are 0.0001 sec)
        period_count = 0;                                 //   rezero counter only at edge, nowhere else!
        if ( (period > 50) && (period < 10000) ) {        //   if we've counted between 50-10000 iter --> rotor speed between 2000-10 rpm
          freq = 10240000 / period;                       //     compute freq (1024x actual) = 1024/(period in seconds) = 1024/(period/10000) = 10240000/period
          rpm = (freq * 60) / parm_alt_poles.intVal();    //     compute RPM (1024x actual), use () for integer math
        }
      } else {                                            // otherwise, no edge has been detected yet...
        if (period_count < 20000) {                       //   if less than 2 sec has elapsed...
          period_count++;                                 //     increment counter, reaches max = 20000
        } else {                                          //   otherwise, it's been too long, so...
          rpm = 0;                                        //     set rpm to zero
        }
      }
      
      // Finally, feed calculated RPM into AnalogChannelBase.
      // We're outside the above if(), so we're calling setInstantaneousValInt() at the same rate as sample_and_test() = 10000 Hz. 
      // As such, the median and low-pass filters in the parent class WILL do their job properly.
      setInstantaneousValInt(rpm, true, true, true);  // do median, do rectify, do low-pass
    }  // END sample_and_test()
    
    int getRPM() {
      return( rpm );
    }
};  // END class RPMChannel


// class RPM2Channel EXTENDS the AnalogChannelBase class.
// This class is used ONLY for channel ac_rpm which is instantiated below as: RPM2Channel ac_rpm(0, 1, 2, "RPM", 120);
// The sample_and_test() function is called below at the full ADC rate (= 10000 Hz) and does the following:
//   1. reads analog channels V1, V2 and V3, 
//   2. uses hysteresis to reject noise and create a square (0 or 1) wave,
//   3. computes freq from the period between square wave leading edges.
//   4. averages the freq from 3 channels
// Why use this channel rather than the Vdiff channel above? Per cw:
//   "Doing hysteresis on diff channel looks the right direction but builds in the jitter between channels."
//   "Probably get less jitter by looking at the three channels independently, then averaging."
class RPM2Channel: public AnalogChannelBase {
  private:
    int channel_num1;
    int channel_num2;
    int channel_num3;
    int hysteresis;
    filter3DBInt filt1;  // separate instances are needed because of recursive calculation
    filter3DBInt filt2;  
    filter3DBInt filt3;
    int rpm[3];

  public:
    RPM2Channel(int num1, int num2, int num3, char* channel_name, int hyst): 
                channel_num1(num1), channel_num2(num2), channel_num3(num3), 
                AnalogChannelBase(channel_name), 
                hysteresis(hyst), 
                filt1( (int)(FREQCHANNEL_ALPHA * 1024.0), false ),  // args are: alpha*1024, take abs(int) - see class filter3DBInt
                filt2( (int)(FREQCHANNEL_ALPHA * 1024.0), false ),  
                filt3( (int)(FREQCHANNEL_ALPHA * 1024.0), false ) {
    }

    void sample_and_test() {
      static int filtered_val[3] = { };
      static int squared_state[3] = { };
      static int last_squared_state[3] = { };
      static int period_count[3] = { };
      int period = 0;
      int freq = 0;

      // read all 3 AC voltage channels ***at as nearly the same moment as possible***
      int raw_val0 = analogRead(channel_num1);  // raw val is 0-4095
      int raw_val1 = analogRead(channel_num2);
      int raw_val2 = analogRead(channel_num3);

      // filter (raw vals*1024) with separate instances of filter3DBInt
      filtered_val[0] = filt1.doFilter(raw_val0 << 10) >> 10;  // multiply arg by 1024x, apply FREQCHANNEL_ALPHA, divide result by 1024x
      filtered_val[1] = filt2.doFilter(raw_val1 << 10) >> 10;
      filtered_val[2] = filt3.doFilter(raw_val2 << 10) >> 10;

      // now... loop through the channels using the filtered voltages
      for (int i = 0; i < 3; i++) {
      
        // Use hysteresis to create a square wave (either 1 or 0)
        last_squared_state[i] = squared_state[i];   
        if (squared_state[i] == 0) {
          if (filtered_val[i] > hysteresis) squared_state[i] = 1;  // if we're definitely (+), hyst = 120 * VOLTAGE_SCALE_FACTOR = 20.7V
        } else {
          if (filtered_val[i] < 60) squared_state[i] = 0;          // if we're below noise threshold ~ 60 * VOLTAGE_SCALE_FACTOR = 10.3V
        }

        // Some relevant rotor speeds:
        // 2000 rpm = 33.3 rev/s * 6 cycles/rev = 200 Hz = 200 cycles/s --> period = 0.005 s/cycle --> *10000 iter/s = 50 iter/cycle = period_count (HIGH freq cutoff)
        //  600 rpm = 10 rev/s * 6 cycles/rev = 60 Hz = 60 cycles/s --> period = 0.0167 s/cycle --> *10000 iter/s = 167 iter/cycle (max rotor speed)
        //  300 rpm = 5 rev/s * 6 cycles/rev = 30 Hz = 30 cycles/s --> period = 0.0333 s/cycle --> *10000 iter/s = 333 iter/cycle (typ. rotor speed)
        //   20 rpm = 0.333 rev/s * 6 cycles/rev = 2 Hz = 2 cycles/s --> period = 0.500 s/cycle --> *10000 iter/s = 5000 iter/cycle (LOW freq cutoff)
        //   10 rpm = 0.167 rev/s * 6 cycles/rev = 1 Hz = 1 cycles/s --> period = 1.000 s/cycle --> *10000 iter/s = 10000 iter/cycle (LOW freq cutoff)
              
        if ( (squared_state[i] - last_squared_state[i]) == 1 ) {  // if we have a positive edge, i.e., squared_state == 1 and last_squared_state == 0
          period = period_count[i];                               //   save measured period (units are 0.0001 sec)
          period_count[i] = 0;                                    //   rezero counter, only at edge, nowhere else!
          if ( (period > 50) && (period < 10000) ) {              //   if we've counted between 50-10000 iter --> rotor speed between 2000-10 rpm
            freq = 10240000 / period;                             //     compute freq (1024x actual) = 1024/(period in seconds) = 1024/(period/10000) = 10240000/period
            rpm[i] = (freq * 60) / parm_alt_poles.intVal();       //     compute RPM (1024x actual), use () for integer math
          }
        } else {                                                  // otherwise, no edge has been detected yet...
          if (period_count[i] < 20000) {                          //   if less than 2 sec has elapsed...
            period_count[i]++;                                    //     increment counter, reaches max = 20000
          } else {                                                //   otherwise, it's been too long, so...
            rpm[i] = 0;                                           //     set rpm to zero
          }
        }
      }  // END loop through 3 channels
      
      // Finally, feed *average* RPM into AnalogChannelBase.
      // We can get away with averaging like this because rpm[i] is not updated UNTIL a valid value has been computed.
      // We're outside the above if(), so we're calling setInstantaneousValInt() at the same rate as sample_and_test() = 10000 Hz. 
      // As such, the median and low-pass filters in the parent class WILL do their job properly.
      setInstantaneousValInt( ((rpm[0]+rpm[1]+rpm[2])/3), true, true, true );  // do median, do rectify, do low-pass
    }  // END sample_and_test()
    
    int getRPM() {
      return( (rpm[0]+rpm[1]+rpm[2]/3) );
    }
};  // END class RPM2Channel


// class PLLChannel EXTENDS the AnalogChannelBase class.
// This class is used ONLY for freq channel ac_pll, instantiated below as: PLLChannel ac_pll(0, 1, "RPM", 200);
class PLLChannel: public AnalogChannelBase {
  private:
    int channel_num1;
    int channel_num2;
    int hysteresis;
    filter3DBInt filt1;
    filter3DBInt filt2;
    int rpm;
    int thediff;
    int filt_val;
    int pll_integral;
    int phase_int;
    int period;
    int iter;

  public:
    PLLChannel(int num1, int num2, char* channel_name, int hyst): 
               channel_num1(num1), channel_num2(num2), 
               hysteresis(hyst), 
               AnalogChannelBase(channel_name), 
               filt1( (int)(0.386 * 1024.0), false ),  // 0.036 --> -3dB @ 60 Hz, consistent 'dropouts' at ~19 Hz = 190 rpm
                                                       // 0.112 --> -3dB @ 200 Hz, consistent dropouts at ~34 Hz = 340 rpm
                                                       // 0.239 --> -3dB @ 500 Hz, observed 1 dropout at ~28 Hz
                                                       // 0.386 --> -3dB @ 1000 Hz, OK so far... up to ~46 Hz = 460 rpm
               filt2( (int)(0.386 * 1024.0), false )  {  // NOT USED, see comment below
    }

    void doPLL() {
      const int sample_rate = SAMPLE_RATE_PER_SEC;  // ADC sampling rate (Hz)
      const int pll_cf = 30;                        // PLL center freq (Hz)
      const int pll_loop_gain = 1;                  // 
      const int TWO_PI_int = TWO_PI*1024.0;         // 2*pi, 1024x actual
      const int PI_int = PI*1024.0;                 //   pi, 1024x actual
      static int sig_state = -1;                    // signal square wave state (-1 or 1)
      static int last_sig_state = -1;               // 
      static int ref_state = 1;                     // reference square wave state (-1 or 1)
      static int last_ref_state = 1;                //
      static int period_count = 0;                  // ref period counter
      int pll_loop_control = 0;                     // = sig * ref
      int raw_val1 = 0;                             // raw channel vals
      int raw_val2 = 0;                             // 
      //int filt_val = 0;                             // filtered channel val
      //int phase_int = 0;                            // ref phase, 1024x actual
      //int period = 0;                               // ref period
      //static int iter = 0;                          // iteration counter
      //static int pll_integral = 0;                  // 'modulation integral' = sum of pll_loop_control vals

      // Limit iterations and pll_integral.
      // Setting the ph(t) equation below = 2*PI gives the following if() condition.
      // Satisfying the if() allows us to rezero the summation vars without perturbing ph(t).
      // Arbitrarily rezeroing these vars (e.g., when iter exceeds some large value) causes spikes in the low freq range!
      if ( (iter + pll_integral) >= (sample_rate/pll_cf) ) {
        iter = 0;
        pll_integral = 0;
      }

      iter++;

      raw_val1 = analogRead(channel_num1);  // raw val is 0-4095 counts --> *0.1723 V/count = 0-705.6V
      raw_val2 = analogRead(channel_num2);
      thediff = raw_val1 - raw_val2;

      // Apply low-pass pre-filter.
      // This filter *significantly* reduces low-rpm spikes.
      filt_val = filt1.doFilter(thediff << 10) >> 10;

      // Create a square wave (-1 or 1) from the AC voltage signal
      // VOLTAGE_SCALE_FACTOR = 0.1723 V/count
      last_sig_state = sig_state;   
      if (sig_state == -1) {
        if ( filt_val > (hysteresis/2) ) sig_state = 1;  // hysteresis = 200 --> 17.23V threshold
      } else {
        if ( filt_val < -(hysteresis/2) ) sig_state = -1;
      }

      // Phase detector, a x b = a - b and a + b
      // See https://arachnoid.com/phase_locked_loop/index.html
      // pll_loop_control IS the modulation function, m(t). See comments below. 
      // Because -1 <= m(t) <= 1, pll_loop_gain CANNOT be set > 1. Doing so disrupts tracking completely!
      // Note also... because both sig and ref are (-1 or 1), their product is also (-1 or 1).
      pll_loop_control = sig_state * ref_state * pll_loop_gain;
      
      // OPTIONAL: Apply a low-pass 'output' filter to select a-b, filter out a+b.
      // If applied, the filter will introduce some phase shift affecting PLL ability to track, 
      // see https://arachnoid.com/phase_locked_loop/index.html
      // ***This doesn't work with square waves!***
      //pll_loop_control = filt2.doFilter(pll_loop_control << 10) >> 10;  // alpha is explicitly set above
      
      // Compute the 'modulation integral' numerator used in the ph(t) equation below.
      pll_integral += pll_loop_control;

      // Generate a frequency-modulated reference square wave (-1 or 1).
      // For (co)sinusoidal modulation, ref = cos(2*PI * pll_cf * (t + pll_integral)), where
      //   t = iter/sample_rate, and pll_integral += pll_loop_control/sample_rate.
      //   See https://arachnoid.com/phase_locked_loop/index.html
      // This equation is closely related the form derived in this reference:
      //   https://classes.engineering.wustl.edu/ese331/LF7/3310_&_331_Project7.pdf
      //
      // The commentary below reconciles these two references.
      // Starting with the fundamental FM equation:
      //   fi(t) = fc + df * m(t), where fi = instantaneous freq, fc = center (carrier) freq, and -1 <= m(t) <= 1 is a modulating function.
      // Let df (max deviation of fm from fc) = fc, where fm = modulation freq, then...
      //   *** fi(t) = fc + fc * m(t) = fc * (1 + m(t)) ***
      // This equation gives the instantaneous freq of an FM wave which can vary from 0 to 2*fc (from m(t) = -1 to m(t) = 1).
      //
      // A sinusoidal, time-varying voltage waveform is described by: 
      //   v(t) = V*sin[ph(t)] or V*cos[ph(t)], where ph(t) = phase angle of the FM wave at time t.
      // The derivative of ph(t) is the rate of change of phase wrt time = instantaneous freq.
      // Think about it like this: if the phase angle isn't changing (zero derivative), the instantaneous freq is zero, and
      //                           if the phase angle is changing rapidly, the instantaneous freq is high.
      //   d[ph(t)]/dt = 2*pi*fi(t)
      // Integrating, we have:
      //   *** ph(t) = integral(2*pi*fi(t)) ***
      //
      // Combining the equations for fi(t) and ph(t), we have:
      //   ph(t) = integral(2*pi*fi(t)) = 2*pi*fc * (t + integral[m(t)dt]), where
      //   m(t)*dt = {a-b}(t)*dt = pll_loop_control * dt, where dt = 1/sample_rate, so...
      //   integral[m(t)*dt] = pll_integral / sample_rate, and
      //   t = iter / sample_rate, giving the result:
      //   *** ph(t) = 2*pi*fc * (iter + pll_integral) / sample_rate ***
      //
      // Constrain reference oscillator phase angle to one cycle: 0 - 2*PI.
      // We use TWO_PI_int here for integer math, so remainder is 1024x actual phase angle.
      // abs() avoids % of negative int. If omitted, there are frequent 'dropouts' to zero rpm.
      phase_int = abs( (TWO_PI_int * pll_cf * (iter + pll_integral)) / sample_rate ) % TWO_PI_int;  // () are for integer math

      // Now, if we had a sinusoidal voltage, e.g., v(t)=V*cos[ph(t)], we'd compute v(t) as such, but...
      // because we're dealing with square waves, we only want the *sign* of sin[ph(t)] or cos[ph(t)], so...
      //   when sin[ph(t)] > 0 --> when ph(t) < PI --> set ref = 1
      //   when sin[ph(t)] < 0 --> when ph(t) > PI --> set ref = -1
      // Interestingly, it turns out we can divide the 2*PI interval in half (arbitrarily!) to generate the reference square wave.
      // Not sure why this is (yet), so we just do it the easiest way:
      last_ref_state = ref_state;  // save last ref state, needed below for determining freq
      if ( phase_int < PI_int ) {  // if phase*1024 < PI*1024
        ref_state = 1;             //   set ref_state = 1
      } else {                     // otherwise...
        ref_state = -1;            //   set ref_state = -1
      }

      // Compute RPM from ref freq, where freq = sample_rate/period.
      if ( (ref_state == 1) && (last_ref_state == -1) ) {                        // if a leading edge has been detected...
        period = period_count;                                                   //   save the period
        period_count = 0;                                                        //   rezero counter
        if ( (period > 100) && (period < sample_rate) ) {                        //   if 100 Hz > freq > 1 Hz --> 1000 rpm > RPM > 10 rpm
          rpm = (1024 * sample_rate * 60) / (period * parm_alt_poles.intVal());  //     1024x actual, () are for integer math
        }
      } else {                                                                   // otherwise, no leading edge has been detected yet...
        if (period_count < sample_rate) {                                        //   if less than 1 sec has elapsed...
          period_count++;                                                        //   increment counter
        } else {                                                                 // otherwise, it's been too long, so...
          rpm = 0;                                                               //   set rpm to zero
        }
      }

      // Finally, feed calculated RPM into AnalogChannelBase.
      // We're outside the above if(), so we're calling setInstantaneousValInt() at the same rate as doPLL() = 10000 Hz. 
      // As such, the median and low-pass filters in the parent class WILL do their job properly.
      setInstantaneousValInt( rpm, true, true, true );  // do median, do rectify, do low-pass
    }  // END doPLL()

    int getDiff() {
      return( thediff );
    }
    int getFiltVal() {
      return( filt_val );
    }
    int getPLLIntegral() {
      return( pll_integral );
    }
    int getIter() {
      return( iter );
    }
    int getPhase() {
      return( phase_int );
    }    
    int getPeriod() {
      return( period );
    }
    int getRPM() {
      return( rpm );
    }
};  // END class PLLChannel

// END class definitions


// BEGIN class instantiations

// Create instances of the AnalogChannel class for each of 9 raw analog inputs.
// These channel vals are scaled by the appropriate _SCALE_FACTOR arg *and* multiplied by 1024 when the read() method is called.
AnalogChannel analog_channels[9] = {
  AnalogChannel(ADC0, VOLTAGE_SCALE_FACTOR, "V1"), 
  AnalogChannel(ADC1, VOLTAGE_SCALE_FACTOR, "V2"),
  AnalogChannel(ADC2, VOLTAGE_SCALE_FACTOR, "V3"),
  AnalogChannel(ADC3, VOLTAGE_SCALE_FACTOR, "VDC"),
  AnalogChannel(ADC4, LINE_VOLTAGE_SCALE_FACTOR, "VL"),
  AnalogChannel(ADC8, CURRENT_SCALE_FACTOR, "I1"),
  AnalogChannel(ADC9, CURRENT_SCALE_FACTOR, "I2"),
  AnalogChannel(ADC10, CURRENT_SCALE_FACTOR, "I3"),
  AnalogChannel(ADC11, CURRENT_SCALE_FACTOR, "IDC")
  };

// Create instances of the AnalogChannelBase class for these channels:
// These channel vals are set in various places. Except for Pd and TP, ALL are 1024x actual.
// TO ADD A NEW CHANNEL:
//   1. Create a new channel, ac_name("Name") here.
//   2. Add the new channel to the acs[] array a bit below this.
//   3. Assign the new channel a const name and value above, e.g., const int WATT_HOURS = 19
//   4. Increase the NUM_ADC_CHANNELS const just after the above list of const channel name = val.
//   5. Edit the web monitor and log file "channels" line to view the new channel.
AnalogChannelBase ac_wind("WS");  // wind speed, 1024x actual
AnalogChannelBase ac_Tr("Tr");    // rectifier temp, 1024x actual
AnalogChannelBase ac_Ta("Ta");    // anemometer/ambient temp, 1024x actual
AnalogChannelBase ac_Tc("Tc");    // controller board temp, 1024x actual
AnalogChannelBase ac_TP("TP");    // tail position, usteps
AnalogChannelBase ac_Pd("Pd");    // nominally 'dump load power', but used for various other parms. TEMPORARY.
                                  // This channel could be renamed at some point when it finds a permanent use.
AnalogChannelBase ac_Wh("Wh");    // wind watt-hours, 1024x actual
AnalogChannelBase ac_DL("DL");    // HVDL duty cycle, 1024x actual

// Create instances of the AnalogDiffChannel class (for each of 3 phase-to-phase voltages):
AnalogDiffChannel l1l2_diff (&analog_channels[0], &analog_channels[1], "V12");
AnalogDiffChannel l2l3_diff (&analog_channels[1], &analog_channels[2], "V23");
AnalogDiffChannel l3l1_diff (&analog_channels[2], &analog_channels[0], "V31");

// Create instances of various RPM channels:
// CHOOSE which ONE of these actually gets used for RPM calculation by:
//   1. calling the relevant class method at the full ADC rate in readADCs(), and
//   2. inserting it into the acs[] definition immediately below.
// Args are: channel(s), channel name, hysteresis - see corresponding method for details
// Hysteresis vals have been arrived at by experimentation to minimize low freq noise.
RPMChannel ac_freq(0, 1, "RPM", 200);     // ac_freq.sample_and_test() method is called below
RPM2Channel ac_rpm(0, 1, 2, "RPM", 120);  // ac_rpm.sample_and_test() method is called below
PLLChannel ac_pll(0, 1, "RPM", 200);      // ac_pll.doPLL() method is called below

// Create an array containing ALL THE ANALOG CHANNELS.
// THIS IS AN IMPORTANT ARRAY! 
// It is the data source for the getChannelRMSInt() function which is called in furlctl() and furlctl1() to control the turbine.
// ***As noted above, insert the desired RPM channel into the acs[] definition.***
AnalogChannelBase* acs[] = { &analog_channels[0], &analog_channels[1], &analog_channels[2], 
                             &analog_channels[3], &analog_channels[4], &analog_channels[5], 
                             &analog_channels[6], &analog_channels[7], &analog_channels[8], 
                             &l1l2_diff, &l2l3_diff, &l3l1_diff, &ac_freq, &ac_wind, 
                             &ac_Pd, &ac_Tr, &ac_Ta, &ac_Tc, &ac_TP, &ac_Wh, &ac_DL };

// END class instantiations


// BEGIN function definitions

// This function sets the various *current* DC offsets and is called in wwe.ino
void initADCOffsets() {
  analog_channels[L1_CURRENT].setDCOffset(CURRENT_OFFSET);
  analog_channels[L2_CURRENT].setDCOffset(CURRENT_OFFSET);
  analog_channels[L3_CURRENT].setDCOffset(CURRENT_OFFSET);
  analog_channels[DC_CURRENT].setDCOffset(CURRENT_OFFSET);
  l1l2_diff.setCrossingReference(10.0);

  // Set filtering constants for individual analog channels; alpha = 1 --> no filtering
  ac_freq1.setAlpha(0.05);  // ac_freq1 is the *original* RPM channel (class AnalogChannelBase); NO LONGER USED (cw had this at 0.05)
  ac_freq.setAlpha(0.036);  // ac_freq is an RPM channel (class RPMChannel) with sampling at 10000 Hz.
  ac_rpm.setAlpha(0.036);   // ac_rpm is an RPM channel (class RPM2Channel) with sampling at 10000 Hz.
  ac_pll.setAlpha(0.036);   // ac_pll is an RPM channel (class PLLChannel) with sampling at 10000 Hz.
  
  ac_wind.setAlpha(1.0);    // anemometer data are already filtered (in base unit), so DON'T APPLY.
  ac_TP.setAlpha(1.0);      // we always want to know the *exact* tail position, so DON'T APPLY.
  ac_Ta.setAlpha(0.06);     // temps change *slowly*, so we use long time constant filtering to smooth and remove noise
  ac_Tc.setAlpha(0.06);
  ac_Tr.setAlpha(0.06);
}


// This CENTRAL FUNCTION is called by a timer interrupt at 10000 Hz - see wwe.ino.
// With each function call, we read *one* of the 9 A/D converters, so we are sampling each of them at 1000 Hz.
// Every tenth function call, voltage diff channels are calculated, so we are sampling the diffs also at 1000 Hz.
// readADCs() analog   channel
// timeslot   channel  name
// 0          A0       V1
// 1          A1       V2
// 2          A2       V3
// 3          A3       VDC
// 4          A4       VL
// 5          A8       I1
// 6          A9       I2
// 7          A10      I3
// 8          A11      IDC
// 9          voltage diff channels, WS channel, temperature channels, TP channel, other stuff...
void readADCs() {
  if (disable_adc) return;
  
  static int call_wind_energy = 0;
  static int wind_Wh = 0;
  static int call_manage_stepper = 0;
  static int call_furlctl = 0;
  static int call_manage_furl1 = 0;
  static int adc_index = 0;
  float frequency = 0.0;
  static int post_count = 0;
  boolean even_second = false;
  
  // EXACTLY ONCE PER SECOND... 
  // POST operating variables to server, start web activity, and do config cycles (less often).
  if ( ++post_count >= SAMPLE_RATE_PER_SEC ) {      // SAMPLE_RATE_PER_SEC = 10000 (defined in wwe.ino)
    myunixtime++;                                   // increment myunixtime by 1 second
    even_second = true;                             // "even" second means EXACTLY 1 second has passed... no stray usec
    post_count = 0;                                 // rezero counter
    ledOn = !ledOn;                                 // toggle the LED boolean
    digitalWriteDirect(TIMER_LOOP_LED_PIN, ledOn);  // on 1s, off 1s, on 1s, off 1s, ...
    do_post = true;                                 // POST flag, used in wwe.ino if (do_post) {} 
  } else {
    even_second = false;
  }

  // NOTE: Unlike the following freq channels, there's no function for the *original* ac_freq1 channel to call below because that channel does not have
  //       a class of its own. Instead, it is managed within the L1L2 diff channel (see class AnalogDiffChannel) by calling l1l2_diff.CalcDiff(true). 
  //       And, since ac_freq1 has been *superseded* by ac_freq, we now call l1l2_diff.CalcDiff(false), just like the other diff channels - see below.

  // Update RPM channels at full ADC rate = 10000 Hz.
  ac_freq.sample_and_test();  // see class RPMChannel ***IN USE ***
  //ac_rpm.sample_and_test();   // see class RPM2Channel ***WORKING***
  //ac_pll.doPLL();             // see class PLLChannel ***WORKING***

  // Manage dump load at full ADC rate = 10000 Hz.
  manageDumpLoad();

  // *******************************************************************************************************
  // * Divide the main 10000 Hz ADC timer into 10 time slots (adc_index==0 to 9), each running at 1000 Hz. *
  // *******************************************************************************************************
  // The first 9 time slots (adc_index==0-8) are used for reading PHYSICAL A/D converters. 
  // Other chores are done in various time slots as well, just to spread out the processing.
  // The 10th time slot (adc_index==9) is used for things like calculating diffs between channels.
  //
  if (adc_index < 9) {
    // Commented out the original line below and added 9 lines, one for each channel read().
    // Doing this allows turning off individual channels by commenting out a line - such as AC line voltage.
    //analog_channels[adc_index].read();                       // read analog channels 0 through 8
    if (adc_index == 0) analog_channels[adc_index].read();  // A0 - L1 voltage
    if (adc_index == 1) analog_channels[adc_index].read();  // A1 - L2 voltage
    if (adc_index == 2) analog_channels[adc_index].read();  // A2 - L3 voltage
    if (adc_index == 3) analog_channels[adc_index].read();  // A3 - DC voltage
    //if (adc_index == 4) analog_channels[adc_index].read();  // A4 - AC line voltage
    if (adc_index == 5) analog_channels[adc_index].read();  // A8 - L1 current
    if (adc_index == 6) analog_channels[adc_index].read();  // A9 - L2 current
    if (adc_index == 7) analog_channels[adc_index].read();  // A10 - L3 current
    if (adc_index == 8) analog_channels[adc_index].read();  // A11 - DC current

    // --> Do these additional tasks when adc_index==0 <--
#ifdef ENABLE_LINEAR_ACTUATOR
    // readADCs() is called at SAMPLE_RATE_PER_SEC = 10000 Hz.
    // adc_index==0 becomes true at 1000 Hz --> max allowable value for FURLCTL_PER_SEC is 1000.
    //   if FURLCTL_PER_SEC = 1000, then furlctl() will be called when call_furlctl == 10000/(10*1000) = 1
    //   if FURLCTL_PER_SEC =   10, then furlctl() will be called when call_furlctl == 10000/(10*10) = 100
    if (adc_index == 0) {
      if ( ++call_furlctl >= (SAMPLE_RATE_PER_SEC / (10 * FURLCTL_PER_SEC)) ) {
        call_furlctl = 0;
        furlctl(even_second);
      }
    }
#endif

    // --> Do these additional tasks when adc_index==1 <--
    if (adc_index == 1) {
      if ( call_wind_energy >= (SAMPLE_RATE_PER_SEC / 10) ) {                                  // at exact 1 sec intervals = every 10000/10 = 1000 iterations
        call_wind_energy = 0;                                                                  //   rezero iter counter
        wind_Wh += (getChannelRMSInt(DC_VOLTAGE)*getChannelRMSInt(DC_CURRENT)) / (3600*1024);  //   cumulative daily watt-sec --> Wh, 1024x actual value
        ac_Wh.setInstantaneousValInt(wind_Wh, false, false, false);                            //   save channel val, DON'T median, DON'T rectify, DON'T filter
      }
      if ( (myunixtime % 86400) == 0 ) {  // at 00:00 UTC...
        wind_Wh = 0;                      //   rezero Wh counter
      }
      call_wind_energy++;  // increment iter counter
    }

    
    // --> Do these additional tasks when adc_index==2 <--
#ifdef ENABLE_STEPPER
    // Call stepper motor state machine.
    // readADCs() is called at SAMPLE_RATE_PER_SEC = 10000 Hz, so...
    // adc_index==2 becomes true at 1000 Hz --> max allowable value for STEPPERCTL_PER_SEC is 1000.
    //   if STEPPERCTL_PER_SEC = 1000, then motor.update* will be called when call_manage_stepper == 10000/(10*1000) = 1
    //   if STEPPERCTL_PER_SEC =   10, then motor.update* will be called when call_manage_stepper == 10000/(10*10) = 100
    if (adc_index == 2) {
      if ( ++call_manage_stepper >= (SAMPLE_RATE_PER_SEC / (10 * STEPPERCTL_PER_SEC)) ) {
        call_manage_stepper = 0;
        motor.updateDecelX();
        motor.updateState();
      }
    }
#endif
    
    // --> Do these additional tasks when adc_index==3 <--
#ifdef ENABLE_STEPPER
    // Call furlctl1() to manage furling with stepper motor.
    // readADCs() is called at SAMPLE_RATE_PER_SEC = 10000 Hz, so...
    // adc_index==3 becomes true at 1000 Hz  --> max allowable value for FURLCTL1_PER_SEC is 1000.
    //   if FURLCTL1_PER_SEC = 1000, then furctl1() will be called when call_manage_furl == 10000/(10*1000) = 1
    //   if FURLCTL1_PER_SEC =  10, then furlctl1() will be called when call_manage_furl == 10000/(10*10) = 100
    if (adc_index == 3) {
      if ( ++call_manage_furl1 >= (SAMPLE_RATE_PER_SEC / (10 * FURLCTL1_PER_SEC)) ) {
        call_manage_furl1 = 0;
        furlctl1();
      }
    }
#endif
    
    // --> Do these additional tasks when adc_index==4 <--
    if (adc_index == 4) {
      // NOTHING YET...
    }
    // --> Do these additional tasks when adc_index==5 <--
    if (adc_index == 5) {
      // NOTHING YET...
    }
    // --> Do these additional tasks when adc_index==6 <--
    if (adc_index == 6) {
      // NOTHING YET...
    }

    // --> Do these additional tasks when adc_index==7 <--
    // Read a reference voltage (Vref) from the CTs on analog input A5 and set DC offsets for the current channels.
    // When no current is flowing through the CT, we should see Vref on the output of the transducer.
    // The Vref signal is NOT divided down, so we scale the Vref down accordingly.
    // Also, we still see about 0.25A of residual current, so we add a small offset to analogRead(A5) as well:
    //   0.25A * 0.04167 V/A * 4095/3.3 counts/V = 13 counts
   if (adc_index == 7) {
      actual_current_offset = (analogRead(A5) - 13) * -13.3/(6.81 + 13.3);
      analog_channels[L1_CURRENT].setDCOffset(actual_current_offset);
      analog_channels[L2_CURRENT].setDCOffset(actual_current_offset);
      analog_channels[L3_CURRENT].setDCOffset(actual_current_offset);
      analog_channels[DC_CURRENT].setDCOffset(actual_current_offset);
    }

    // --> Do these additional tasks when adc_index==8 <--
    if (adc_index == 8) {
      a7_val = analogRead(A7);  // read controller rectifier thermistor temp, see calcThermistorTemp() in utils.ino
    }

  } else {  // --> Do these tasks when adc_index==9 <--

    // Process reed switch state
    // We should probably implement the reed switch debounce business as a new class...
    // but for now... this function sets a global var: debounced_rs_state
    debounceRS();

    // Process L1-L2, L2-L3, L3-L1 voltage diff channels
    // If the CalcDiff arg ==true, the diff channel is used to calculate frequency and store it in ac_freq1 - see class AnalogDiffChannel.
    // This was the original frequency determination routine, *superseded* by ac_freq - see class RPMChannel.
    l1l2_diff.CalcDiff(false);  // <-- this was set true when we were using this diff channel for freq determination
    l2l3_diff.CalcDiff(false);
    l3l1_diff.CalcDiff(false);

    // Process wind speed channel (WS)
    // We're sending wind speed vals that are updated at only 1 Hz, to be filtered at 1000 Hz.
    // As such, there will be about 1000 identical vals set before a new val appears, so the filters will have almost no effect on such low freq data.
    ac_wind.setInstantaneousValInt(windspeed_ms, true, true, true);  // m/s 1024x actual; DO median, DO rectify, DO low-pass
    
    // Process temperature channels (Tc, Ta, Tr)
    ac_Tc.setInstantaneousValInt(Tctl, true, false, true);                // controller board temp (degC), Tctl is 1024x actual (see wwe.ino), do median, DON'T rectify, do filter
    ac_Ta.setInstantaneousValInt(Ta, true, false, true);                  // anemometer temp (degF), Ta is 1024x actual (see wind.ino), do median, DON'T rectify, do filter
    ac_Tr.setInstantaneousValInt(rectifier_temp_int, true, false, true);  // rectifier temp (degC), rectifer_temp_int is 1024x actual (see wwe.ino), do median, DON'T rectify, do filter
    
    // Process tail position (TP) channel
    ac_TP.setInstantaneousValInt(motor.currentPosition(), false, false, false);  // usteps, NOT 1024x actual; DON'T median, DON'T rectify, DON'T low-pass

    // Process dump load duty cycle (HVDL_DUTY_CYCLE) channel
    ac_DL.setInstantaneousValInt(dump_load_duty_cycle, false, false, false);  // integer percentage, NOT 1024x actual; DON'T median, DON'T rectify, DON'T low-pass

    // Process dump load power (Pd) channel
    /*
    // Dump load power = Pd = duty cyle * VDC^2 divided by dump load resistance
    // setInstantaneousValInt() sets a channel value, arg must be multiplied by 1024
    // getInstantaneousValInt() gets a channel value, return value is multiplied by 1024
    int vdc_int = analog_channels[DC_VOLTAGE].getInstantaneousValInt() >> 10;                                          // need actual value, so divide by 1024
    ac_Pd.setInstantaneousValInt( ((dump_load_duty_cycle*vdc_int*vdc_int) / (100*parm_hvdl_R.intVal())) << 10 );  // multiply actual value by 1024
    */

    // Collect waveforms
    // startCollectingWaveforms(), called by waveCmd(), sets collect_waveforms == true
    if (collect_waveforms == true) {
      if ( ++waveform_ptr >= SAMPLES_IN_WAVEFORM ) {
        waveform_ptr = 0;
        collect_waveforms = false;
      }
    }
  }  // END else (adc_index==9)

  // Increment adc_index, or reset adc_index to 0 if it reaches 9  
  adc_index = (adc_index < 9) ? ++adc_index : 0;  
  
}  // END readADCs()




void printAnalogChannels() {
  for (int i = 0; i < 9; i++) {
    Serial << "adc: channel " << i << " = " << analog_channels[i].getChannelNum() << "\n";
  }
}


float *getChannelWaveformData(int channel) {
  //Serial.println(channel);
  return( acs[channel]->getWaveformData() );
}


char* getChannelName(int channel) {
  if (channel == STATE) {
    return("State");
  } else {
    return( acs[channel]->getName() );
  }
}


void printChannelsRMS() {
  for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
    if (i != STATE) {
      float val = (float)getChannelRMSInt(i)/1024.0;
      if (i == TAIL_POSITION) val = round(val * 2.97);  // Special hack to get tail position in deg, for which we stored the raw step count.
      Serial << getChannelName(i) << "=" << val << " ";
      if (i == RPM) Serial << "(" << (val * ((float)parm_alt_poles.intVal()) / 60.0) << " Hz) ";  // parenthetically show RPM as Hz
    } else {  // executes when i == STATE. Assumes that "State" is the last channel! 
      Serial.print("State=");
      Serial.println(getChannelRMSInt(i), HEX);
    }
  }
  // For comparison of original and new RPM channel data
  //Serial << "adc: RPM = " << (ac_freq.getAvgValInt()>>10) << "\n";
  //Serial << "adc: PLL = " << (ac_pll.getAvgValInt()>>10) << "\n";
  //Serial << "adc: thediff = " << ac_pll.getDiff() << "\n";
  //Serial << "adc: filt_val = " << ac_pll.getFiltVal() << "\n";
  //Serial << "adc: iter = " << ac_pll.getIter() << "\n";
  //Serial << "adc: pll_integral = " << ac_pll.getPLLIntegral() << "\n";
  //Serial << "adc: phase = " << ac_pll.getPhase() << "\n";
  //Serial << "adc: period = " << ac_pll.getPeriod() << "\n";
}


// This function returns either a 32-bit int controller state OR a channel value as an int = 1024x its actual value (to facilitate integer math).
// The return value may be median-filtered and/or rectified and/or low-pass filtered, depending on how the channel was read by setInstantaneousValInt().
int getChannelRMSInt(int channel) {
#ifdef USE_TEST_VALS
  return(test_values[channel]);
#else
  if (channel == STATE) {                  // if we want the controller state channel
    return( getControllerState() );         //   get controller state bitfield, 32-bit int
  } else {                                  // otherwise...
    return acs[channel]->getAvgValInt();    //   get channel val, 1024x actual
  }
#endif
}


void setTestValue(int index, float value) {
  if (index < 14) {
    dbgPrint(1, "adc setTestValue(");
    dbgPrint(1, index);
    dbgPrint(1, ", ");
    dbgPrint(1, value);
    dbgPrint(1, ")");
    test_values[index] = value;
  }
}


void startCollectingWaveforms() {
  collect_waveforms = true;
}


boolean checkCollectingWaveforms() {
  return collect_waveforms;
}


void debounceRS() {
  // We're calling this above at 1000 Hz = 1 msec intervals
  // Max motor speed is 20 deg/s, so if debounce_interval is 30 msec: 0.03 s * 20 deg/s = 0.6 deg of tail motion
  // Set debounce_interval longer for greater reliability (at least 20 msec according to web research)
  int current_rs_state;
  static int last_rs_state;
  const int debounce_interval = 30;  // msec
  static int ms_unchanged = 0;       // msec counter

  current_rs_state = digitalReadDirect(REED_SWITCH_PIN);  // get current RS state
  if (current_rs_state == last_rs_state) {    // if a transition has NOT occurred...
    if (ms_unchanged < debounce_interval) {   //   if # of msec that RS state has not changed is < desired interval           
      ms_unchanged++;                         //     increment msec counter
    } else {                                  //   otherwise, msec counter HAS exceeded the desired interval
      debounced_rs_state = current_rs_state;  //     so accept current RS state as "debounced"
    }
  } else {                                    // a transition HAS occurred...
    ms_unchanged = 0;                         //   so rezero msec counter
  }
  last_rs_state = current_rs_state;
  // debouncd_rs_state == 1 == RS open --> REED_SWITCH_STATE_LED_PIN == 1 --> LED OFF
  debounced_rs_state ? digitalWriteDirect(REED_SWITCH_STATE_LED_PIN, 1) : digitalWriteDirect(REED_SWITCH_STATE_LED_PIN, 0);
}

// END function definitions
