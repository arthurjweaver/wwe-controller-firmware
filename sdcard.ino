// ---------- sdcard.ino ----------

// From SdFat documentation: 
// "SanDisk cards generally have good write performance. They seem to have more internal RAM buffering than other cards 
// and therefore can limit the number of flash erase operations that the Arduino forces due to its limited RAM."

// SdFat NOTES:
// Arduino DUE is SAM3x, not AVR, so... edit SdFatConfig.h:
// All other boards.
//#define ENABLE_DEDICATED_SPI 0

#define ARDUINO_SAM_DUE  // see SdSpiDue.cpp

// Initialize SD card and print a directory listing.
boolean initializeSDCard() {
  Serial << "sdcard: Initializing SD...\n";
  // SPI_HALF_SPEED arg slows writes from ~37 msec to ~110 msec, but is necessary to prevent write errors! - see SdFat docs
  // The default (w/o arg) is apparently SPI_FULL_SPEED
  // Options apparently are: SPI_HALF_SPEED, _QUARTER_, _EIGHTH_, _SIXTEENTH_
  // "...the standard specifies that the clock must be in the range 100-400 kHz for initialization." - see https://github.com/greiman/SdFat/issues/111
  // Since full speed = 400 MHz, use SPI_HALF_SPEED or SPI_QUARTER_SPEED
  // But, neither of these work to initialize SD after Ethernet startup, so what now?
  //
  if ( !(SD_ok = SD.begin(4, SPI_HALF_SPEED)) ) {  // use this line with SdFat.h
    Serial << "sdcard: SD initialization FAILED.\n";
    return(false);
  } else {
    Serial << "sdcard: SD initialization successful.\n";
    //File rootfile = SD.open("/");
    //printDirectory(rootfile, 2);  // print SD card directory - see below
    //rootfile.close();
    return(true);
  }
}


// Write controller or modbus JSON data to SD.
boolean writeDataToSD(char *fname, int do_modbus) {
  File sdfile;
  String channels_strg = "";
  String vals_strg = "";

  //Serial << "sdcard: fname = " << fname << "\n";

  if ( SD_ok ) {                                              // if SD is OK... (SD_ok is a global var - see setup() when SD card in initialized)
    if ( !SD.exists(fname) ) {                                //   if the file does NOT exist... it's a new day!
      //if ( sdfile = SD.open(fname, FILE_WRITE) ) {            //     if we can open the file...
      if ( sdfile = SD.open(fname, O_CREAT | O_WRITE) ) {
        //Serial << "sdcard: Opened NEW file: " << fname << " (" << sdfile.size() << " bytes)\n";  // .size() method caused compiler errors with SdFat
        channels_strg = getChannelsJSON(do_modbus);           //       generate a channels line for the top of the file
        sdfile.println(channels_strg);                        //       write a channels line
        Serial << "sdcard: Wrote " << channels_strg.length() << " bytes to " << fname << "\n";
        vals_strg = getValsJSON(do_modbus);                   //       generate a vals line
        sdfile.println(vals_strg);                            //       write the first vals line
        Serial << "sdcard: Wrote " << vals_strg.length() << " bytes to " << fname << "\n";
        sdfile.close();                                       //       close the file
      } else {                                                //     otherwise, we can't open the file
        Serial << "sdcard: Error opening " << fname << "\n";  //       print an error
        return(false);                                        //       end in failure.
      }
    } else {                                                  //   otherwise, the file exists...
      //if ( sdfile = SD.open(fname, FILE_WRITE) ) {            //     if we can open the file...
      if ( sdfile = SD.open(fname, O_APPEND | O_WRITE) ) {
        //Serial << "sdcard: Opened EXISTING file: " << fname << " (" << sdfile.size() << " bytes)\n";  // .size() method caused compiler errors with SdFat
        vals_strg = getValsJSON(do_modbus);                   //       generate a vals line
        sdfile.println(vals_strg);                            //       write a vals line
        //Serial << "sdcard: Wrote " << vals_strg.length() << " bytes to " << fname << "\n";
        sdfile.close();                                       //       close the file
      } else {                                                //     otherwise, we can't open it
        Serial << "sdcard: Error opening " << fname << "\n";  //       print an error
        return(false);                                        //       end in failure.
      }
    }
  } else {                                                    // otherwise...
    Serial << "sdcard: SD is NOT OK!\n";                      //   print an error
    return(false);                                            //   end in failure.
  }
  return(true);  // if we make it here, we've been successful!
}



// Generate a JSON string containing Controller or Modbus channels.
String getChannelsJSON(int do_modbus) {
  String channels_strg = "";
  int num_channels;

  if (do_modbus == 0) num_channels = NUM_ADC_CHANNELS;   // 0 == Controller data, NUM_ADC_CHANNELS is defined in adc.ino
  if (do_modbus == 1) num_channels = NUM_MOD_FAST_CHANNELS;  // 1 == Morningstar Modbus/RTU 'fast' regs, NUM_MOD_FAST_CHANNELS is defined in modbus.h
  if (do_modbus == 2) num_channels = NUM_MOD_SLOW_CHANNELS;  // 2 == Morningstar Modbus/RTU 'slow' regs, NUM_MOD_SLOW_CHANNELS is defined in modbus.h
  if (do_modbus == 3) num_channels = NUM_MOD_NUV_CHANNELS;   // 3 == Nuvation Modbus/TCP regs, NUM_MOD_NUV_CHANNELS is defined in modbus.h

  channels_strg = "{\"channels\":[\"";  //   start a channels JSON string

  for (int i = 0; i < num_channels-1; i++) {                       // for each channnel (except the last)...
    channels_strg.concat( getChanName(i, do_modbus) );             //   get channel name - see webclient.ino
    channels_strg.concat( "\",\"" );                               //   continue string
  }
  channels_strg.concat( getChanName(num_channels-1, do_modbus) );  // get last channel
  channels_strg.concat ( "\"]}" );                                 // end string
  return (channels_strg);                                          // return a String object
}


// Generate a JSON string containing Controller or Modbus vals.
String getValsJSON(int do_modbus) {
  String vals_strg = "";
  int num_channels;

  if (do_modbus == 0) num_channels = NUM_ADC_CHANNELS;   // 0 == Controller data, NUM_ADC_CHANNELS is defined in adc.ino
  if (do_modbus == 1) num_channels = NUM_MOD_FAST_CHANNELS;  // 1 == Morningstar Modbus/RTU 'fast' regs, NUM_MOD_FAST_CHANNELS is defined in modbus.h
  if (do_modbus == 2) num_channels = NUM_MOD_SLOW_CHANNELS;  // 2 == Morningstar Modbus/RTU 'slow' regs, NUM_MOD_SLOW_CHANNELS is defined in modbus.h
  if (do_modbus == 3) num_channels = NUM_MOD_NUV_CHANNELS;   // 3 == Nuvation Modbus/TCP regs, NUM_MOD_NUV_CHANNELS is defined in modbus.h

  vals_strg = "{\"vals\":[";  // start a vals JSON string

  for (int i = 0; i < num_channels-1; i++) {                                      // for each channel (except the last)...
    if ( do_modbus > 0 ) {                                                        //   if this is modbus data...
      vals_strg.concat( String( getModchannelValue(do_modbus, i) ) );             //     get the val, returned as char*
    } else {                                                                      //   otherwise, it's controller data...
      float multiplier = 0.0009765625;                                            //     channel values are 1024x actual, so multiply by 1/1024 = 0.0009765625, except for...
      if (i == TAIL_POSITION) multiplier = 0.002903226;                           //     tail position, in usteps, with 360/(2000*62) deg/ustep, so multiplier = 0.002903226
      vals_strg.concat( String( ((float)getChannelRMSInt(i)*multiplier), 2) );    //     show 2 decimal places
      vals_strg.concat( "," );                                                    //     continue string
    }
  }
  if ( do_modbus > 0 ) {                                                          // if this is modbus data...
    vals_strg.concat( String( getModchannelValue(do_modbus, num_channels-1) ) );  //   get last val, returned as char*
  } else {                                                                        // otherwise, it's controller data...
    vals_strg.concat( String( getChannelRMSInt(num_channels-1) ) );               //   last val is State (no multiplier), returned as int
  }
  vals_strg.concat( "], \"time\":" );                                             // end vals array
  vals_strg.concat( String(myunixtime) );                                         // add timestamp
  vals_strg.concat( "}" );                                                        // end string
  return (vals_strg);                                                             // return a String object
}


/*
// This function prints an SD card directory listing.
void printDirectory(File dir, int numTabs) {
   while(true) {
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);  // RECURSION?!
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);  // .size() method caused compiler errors with SdFat
     }
     entry.close();
   }
}
*/
