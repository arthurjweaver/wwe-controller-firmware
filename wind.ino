//---------- wind.ino ----------
// This module is the interface to the wireless Etesian anemometer. 
// In addition to the processSerialWind() method, it provides methods 
// that are used to set the calibration of the device.

void initWind(){
  Serial2.begin(9600);  // Serial2 is the Etesian interface
  delay(2000);
  
  Serial.println("wind: Initializing Etesian");
  Serial2.write(0x1b);  // ESC
  Serial2.flush();
  delay(500);
  Serial2.write(0x1b);  // ESC
  Serial2.flush();
  delay(500);

  // Set the Etesian anemometer data format.
  // We tried NMEA first, but for some reason that didn't work or stopped working with new Etesian firmware.
  // Now we just use the default, so we don't have to set it.
  //Serial2.print("NMEA\r\n");
}


// This function seems not to be used anywhere...
void printEtesian(char* strg) {
  Serial.println(strg);
  Serial2.print(strg);
  Serial2.print("\r\n");
  delay(500);
}


// This is a wind speed median value filter function
// Reminder: wind speeds are 1024x actual
int windvals[3];
int windvalsptr = 0;

int getWindMedianVal(int val){
  windvals[windvalsptr++] = val;
  if(windvalsptr >= 3) windvalsptr = 0; // reset pointer
  if((windvals[0] > windvals[1]) && (windvals[2] > windvals[1])){
    if(windvals[0] <= windvals[2]){
      return(windvals[0]);
    }else{
      return(windvals[2]);
    }
  }else{
    return(windvals[1]);
  }
}


void setWindCalScaleOffset(float offset) {
 // do something 
}


// This function is called every iteration of loop() - see wwe.ino.
// loop() runs at whatever rate it can depending on how much time each timer interrupt cycle requires - see wwe.ino and adc.ino.
//
// The Etesian is initialized into STREAM mode, sending out data at its own rate, somewhere around 1 per second, if the wind is blowing. 
// We want to capture data whenever available, setting windspeed_ms and Ta as global vars. 
// Each time we set these vars, we also save the time... in order to reject non-current data (see below).
void processSerialWind() {
  static char linebuf[200];  // do all these need to be static?
  static char buf[10];
  static char buf1[10];
  static int bufptr = 0;
  static int buf1ptr = 0;
  static int commacount = 0;
  static int bytes_read = 0;

  //Serial << "wind: processSerialWind() called\n");

  // Read from Serial2 as many characters as there are, and grab the ones that represent WS and Ta
  // Default string looks like this: 0/0/0,05:01:57,29,6.6 mph,19.3
  int num_available = Serial2.available(); // find how many chars are available...
  while (num_available-- > 0) {            // while there are chars available...
    int inByte = Serial2.read();           //   read one char
    linebuf[bytes_read++] = inByte;        //   put it in the linebuf[] array
    
    if (inByte == 0x2c) {                  //   is char a COMMA?
      commacount++;                        //     # commas = # fields
      buf[bufptr] = 0;                     //     null terminate buf (WS) array
      buf1[buf1ptr] = 0;                   //     null terminate buf1 (Ta) array
    } else if (commacount == 3) {          //   if we have 3 commas, we've reached the WS field, so...
      buf[bufptr++] = (char)inByte;        //     copy this char to buf, then increment pointer
    } else if (commacount == 4) {          //   if we have 4 commas, we've reached the Ta field, so...
      buf1[buf1ptr++] = (char)inByte;      //     copy this char to buf1, then increment pointer
    }
    
    if (inByte == 0x0a) {                  //   is char a NEWLINE?
      //Serial << "wind: detected newline char\n";
      buf1[buf1ptr] = 0;                   //     null terminate buf1 (Ta) array
      if (commacount == 4) {               //     if we've seen 4 commas (there should only be 4!), process the WS chars
        if (buf[3] == 0x20) {              //       0x20=space between WS # and "mph" could occur after 3 or 4 chars, e.g., 6.7 or 16.7
          buf[3] = 0;                      //         null terminate a 3-char WS string
        } else {                           //       otherwise...
          buf[4] = 0;                      //         null terminate a 4-char WS string
        }
        //Serial "wind: " << buf <<" \n";

        // Now we can process the WS and Ta vals...
        last_windspeed_ms = windspeed_ms;                   // save last WS
        windspeed_ms = int( atof(buf) * 1024.0 * MPH2MS );  // get new WS, WS in buf --> float --> int (1024x actual), mph --> m/s
                

        // REJECT any WS that changes by more than +15 mph from last WS value: 15 mph * 0.44704 (ms/mph) * 1024 = 6867
        // We don't reject a sudden WS decrease (probably to zero) because this may be due to anemometer freezing!
        // We don't use a median filter because that delays WS (even more than it is already).
        if ( (windspeed_ms - last_windspeed_ms) > 6867 ) windspeed_ms = last_windspeed_ms;

        // In sub-zero weather, Ta is displayed (and sent out) as Ta(out) = Ta(sub-zero) + 360. This is fixed below.
        int raw_Ta = 1024 * (int)atof(buf1);                    // Ta string --> float --> int (1024x actual)
        if (raw_Ta > (1024*200)) raw_Ta = raw_Ta - (1024*360);  // if Ta > 200F, subtract 360F from raw val 
        Ta = raw_Ta;                                            // save new Ta
                
      } // END if commacount == 4
      
      linebuf[bytes_read] = 0;  // null terminate linebuf array
      // Seems there's a second new line char between data lines, causing this Serial statement to repeat with no data
      //Serial << "wind: linebuf[] = " << linebuf; // linebuf will have CR/LF, so no "\n" needed here
      
      bytes_read = 0;          // reset char counter
      commacount = 0;          // reset comma counter
      bufptr = 0;              // reset WS (buf) array pointer
      buf1ptr = 0;             // reset Ta (buf1) array pointer
    } // END if new line
    
  } // END while()

  if ( windspeed_ms == 0 ) {                                  // if WS is reported as zero...
    windspeed_ms = int( last_windspeed_ms * exp(-1/300.0) );  //   do exponential decay with time constant = 300s
                                                              //   ***floating point math requires #.0 notation! otherwise (-1/#) = 0 ***
    last_windspeed_ms = windspeed_ms;                         //   save last WS
  }
  
  //Serial << "wind: " << bytes_read << " bytes read\n"; // safety for line buffer overrun
  if (bytes_read > 200) bytes_read = 0;
}

