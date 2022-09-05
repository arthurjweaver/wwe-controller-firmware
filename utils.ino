// ---------- utils.ino ----------
// This module is a collection of functions that don't obviously fit elsewhere.

//
// © Francesco Potortì 2013 - GPLv3 - Revision: 1.13
// This function sends an NTP packet and wait for the response. It returns the Unix time.
//
// To lower the memory footprint, no buffers are allocated for sending and receiving the NTP packets.
// Four bytes of memory are allocated for transmision, the rest is random garbage collected from the data memory segment,
// and the received packet is read one byte at a time. The Unix time is returned, that is, seconds from 1970-01-01T00:00.

// This function is called from setup() in web.ino.
//   ntptime = ntpUnixTime(ntpudp);
//   where ntpudp is an object in class EthernetUDP. In Ethernet.h, a derived class (UDP) inherits the EthernetUDP class.
//   So below, ntpUnixTime is being passed a parameter which is the address of an object of class UDP.
unsigned long inline ntpUnixTime(UDP &udp) {

  static int udpInited = udp.begin(123);  // open socket on an arbitrary CLIENT port
  if ( !udpInited ) return 0;             // FAIL if udp.begin() did not init a socket
  udp.flush();                            // clear received data of possible stray received packets
 
  // Send an NTP request to the NTP port (port 123) on the time server.
  // Only the first 4 bytes of an outgoing NTP request packet need to be set appropriately, the rest can be whatever.
  const char timeServer[] = "pool.ntp.org";                     // NTP server name
  const long ntpFirstFourBytes = 0xEC0600E3;                    // these are the 4 required bytes
  if ( !( udp.beginPacket(timeServer, 123) &&                   // open the packet
          udp.write( (byte *)&ntpFirstFourBytes, 48 ) == 48 &&  // write the required bytes to a 48-byte packet
          udp.endPacket() ) ) return 0;                         // close the packet; if any of that fails, return 0

  // Wait for a response from the time server. Check every pollIntv msec up to maxPoll times.
  const int pollIntv = 150;		                        // poll interval, msec
  const byte maxPoll = 15;		                        // max # of poll attempts
  int pktLen;				                                  // received packet length
  for ( byte i = 0; i < maxPoll; i++ ) {              // for each poll attempt...
    if ( (pktLen = udp.parsePacket()) == 48 ) break;  //   if a 48-byte packet is received, break out of for() loop
    delay( pollIntv );                                //   wait before trying again
  }
  if ( pktLen != 48 ) return 0;  // FAIL if no packet of correct length is received

  // Read and discard the first useless bytes, bytes 0-32 or 0-40.
  const byte useless = 40;  // set to 32 for speed or 40 for accuracy
  for ( byte i = 0; i < useless; ++i ) udp.read();

  // Read the INTEGER part of NTP time (read bytes 41-44, unsigned long is 4 bytes = 32 bits)
  unsigned long NTPtime = udp.read();                                  // read first byte
  for ( byte i = 1; i < 4; i++ ) NTPtime = NTPtime << 8 | udp.read();  // shift last byte left 8 bits, then bitwise OR with next byte

  // For accuracy, round the INTEGER time to the nearest second:
  //   The FRACTIONAL part of NTP time = next byte (0-255) / 256.
  //   So, if (next byte / 256) > 0.5 sec, round up if next byte > 128, but...
  //     assume a network delay = 0.05 sec, so round up if next byte > (0.5 - 0.05) * 256 = 115, but...
  //     assume a further delay in reading the packet since it arrived = pollIntv/10 = 15 msec --> (0.5 - 0.05 - 0.015) * 256 = 111
  NTPtime += ( udp.read() > 111 );  // if byte > arg, add 1 sec; otherwise add 0 sec

  udp.flush();                    // discard the rest of the packet
  // convert NTP time (seconds since 1900-01-01) to unix time (seconds since 1970-01-01)
  return NTPtime - 2208988800ul;  // subtract 70 years = 2208988800 sec
}  // END ntpUnixTime



// This function reads a byte from EEPROM.
//   The EEPROM has 256 bytes. The first 128 are available to us for storage of whatever we want to save,
//   e.g., machine state or operating parameters. The next 128 are read-only, used ONLY to hold a preprogrammed UNIQUE MAC address.
byte readEEPROMReg(byte r) {
  int duration = 0;
  unsigned char v;

  Wire.beginTransmission(I2C_ADDRESS);    // open 
  Wire.write(r);                          // send register to read
  Wire.endTransmission();                 // close
  Wire.requestFrom(I2C_ADDRESS, 1);       // request one byte
  while( !Wire.available() ) {            // if no datum is available...
    delay(10);                            //   wait, but not too long
    duration++;                           //   implement a timeout
    if ( duration > 4 ) return((byte)0);  //   if we timeout, return a 0 byte
  }
  v = Wire.read();  // if we make it here, read one byte
  return v;         // return it
}  // END readEEPROMReg



// This function calculates the VUB72 rectifer temperature.
//   It is called in wwe.ino as: calcThermistorTemp(2200, 3560, 978, 3.3)
//   T0 = 25C = 298.15K, Rt25 = 2200 ohms, B = 3560K, Rs = 978 ohms, Vdd = 3.3V 
//   1/T = ln(Rt/Rt25)*(1/B) + 1/T0
float calcThermistorTemp(int Rt25, int B, int Rs, float Vdd) {
  float Vout = Vdd * ((float)a7_val / 4096.0);                         // output voltage, where A7 register --> a7_val, read at 1000 Hz in adc.ino
  float Rt = Rs * Vout / (Vdd - Vout);                                 // calculate thermistor resistance from Vout
  float T = ( 1.0 / ((log(Rt/Rt25) * 1/B) + (1.0/298.15)) ) - 273.15;  // -273.15 converts K to C
  return(T);
}



// This function toggles a diagnostic LED on/off
void myHandler() {
  ledOn = !ledOn;
  //digitalWriteDirect(myLED, ledOn); // LED on, off, on, off...
}



// ***PRINTING TO SERIAL***
//   Serial runs at 115200 baud, so it takes about 10(bits/char)/115200(bits/sec) = 87 usec/char to get a char out. 
//   Equivalently, about 11520 chars/sec are possible.
//   However, the timer loop - readADCs() - runs at 10,000 Hz (100 usec/iter), so we have to run ALL of the code called by it in this time!
//   This means - NO SERIAL OUTPUT in anything that is called in the timer loop - including the code in furlctl.ino!
//
//   On the other hand... Serial.print() probably just sticks stuff in a queue, so maybe we could get by with it as long 
//   as the action is threadsafe, which we don't know. If the main loop is executing a Serial.print() that is 
//   interrupted by the timer, and print() doesn't handle this, we could have a problem. Best to just avoid it!
//
//
// Debug print functions for: char*, const char*, int, unsigned int, long, unsigned long, double, and boolean
void dbgPrint(int msglvl, char* msg){
  if(msglvl >= MSGLVL) Serial.print(msg);
}
void dbgPrintln(int msglvl, char* msg){
  if(msglvl >= MSGLVL) Serial.println(msg);
}
void dbgPrint(int msglvl, const char* msg){
  if(msglvl >= MSGLVL) Serial.print(msg);
}
void dbgPrintln(int msglvl, const char* msg){
  if(msglvl >= MSGLVL) Serial.println(msg);
}
void dbgPrint(int msglvl, int msg){
  if(msglvl >= MSGLVL) Serial.print(msg);
}
void dbgPrintln(int msglvl, int msg){
  if(msglvl >= MSGLVL) Serial.println(msg);
}
void dbgPrint(int msglvl, unsigned int msg){
  if(msglvl >= MSGLVL) Serial.print(msg);
}
void dbgPrintln(int msglvl, unsigned int msg){
  if(msglvl >= MSGLVL) Serial.println(msg);
}
void dbgPrint(int msglvl, long msg){
  if(msglvl >= MSGLVL) Serial.print(msg);
}
void dbgPrintln(int msglvl, long msg){
  if(msglvl >= MSGLVL) Serial.println(msg);
}
void dbgPrint(int msglvl, unsigned long msg){
  if(msglvl >= MSGLVL) Serial.print(msg);
}
void dbgPrintln(int msglvl, unsigned long msg){
  if(msglvl >= MSGLVL) Serial.println(msg);
}
void dbgPrint(int msglvl, double msg){
  if(msglvl >= MSGLVL) Serial.print(msg);
}
void dbgPrintln(int msglvl, double msg){
  if(msglvl >= MSGLVL) Serial.println(msg);
}
void dbgPrint(int msglvl, boolean msg){
  if(msglvl >= MSGLVL) Serial.print(msg);
}
void dbgPrintln(int msglvl, boolean msg){
  if(msglvl >= MSGLVL) Serial.println(msg);
} 



// REBOOT function - NOT CURRENTLY USED
int binary_exec(void * vStart){
    int i;

    // -- Check parameters
    // Should be at least 32 words aligned
    if ((uint32_t)vStart & 0x7F)return 1;
    // Should in code or sram region
    //if ((uint32_t)vStart > CM3_SRAM_END)
        //return 2;

    // -- Disable interrupts
    // Disable IRQ
    __disable_irq();
    // Disable IRQs
    for (i = 0; i < 8; i ++) NVIC->ICER[i] = 0xFFFFFFFF;
    // Clear pending IRQs
    for (i = 0; i < 8; i ++) NVIC->ICPR[i] = 0xFFFFFFFF;

    // -- Modify vector table location
    // Barriers
    __DSB();
    __ISB();
    // Change the vector table
    SCB->VTOR = ((uint32_t)vStart & SCB_VTOR_TBLOFF_Msk);
    // Barriers
    __DSB();
    __ISB();

    // -- Enable interrupts
    __enable_irq();

    // -- Load Stack & PC
    _binExec(vStart);

    return 0;
}


// Called by binary_exec()
void _binExec (void * l_code_addr){
    __asm__ ("mov   r1, r0        \n"
             "ldr   r0, [r1, #4]  \n"
             "ldr   sp, [r1]      \n"
             "blx   r0"
             );
}
