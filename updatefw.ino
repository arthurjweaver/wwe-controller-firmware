// This is a BOOTLOADER program that is executed to download and start a new version of controller firmware. 
// ***This program MUST be linked to run from Arduino Flash Bank 1 at location 0xC0000 in flash memory.***
//
// This program visits the Update Server and downloads a firmware binary file.
// The firmware binary file is downloaded via HTTP, written to flash Bank 0 at address 0x80000 (the usual 
//   location of Arduino code), and started at that location.
//
// To build this program to execute at 0x80000 with a size not exceeding 0x40000, EDIT:
//   %USERPROFILE%\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\variants\arduino_due_x\linker_scripts\gcc\flash.ld
//
// COMMENT OUT the original line and ADD two lines (also intentionally commented-out):
  /*rom (rx)    : ORIGIN = 0x00080000, LENGTH = 0x00080000*/ /* Flash, 512K */
  /*rom (rx)    : ORIGIN = 0x00080000, LENGTH = 0x00040000*/ /* Flash Bank 0, 256K */
  /*rom (rx)    : ORIGIN = 0x000C0000, LENGTH = 0x00040000*/ /* Flash Bank 1, 256K */
//
// Note the shorter LENGTH = 0x40000 = 256K for both ORIGIN's!
//   This is necessary because wwe.ino writes the bootloader updatefw.bin to 0xC0000, which must NOT be a  
//   location within its own program space, as it would be if LENGTH were 0x80000 = 512K. 
//   See https://stackoverflow.com/questions/47078293/atmel-sam3x-dual-bank-switching-not-working
// THEN...
//   Uncomment the Flash Bank 0 line when creating a compiled binary for wwe.ino, or
//   ***UNCOMMENT the Flash Bank 1 line when creating a compiled binary for updatefw.ino***
//
// Create a compiled binary file: 
//   Sketch --> Export compiled Binary
//
// Open a Cygwin64 Terminal window and copy compiled binary (and source) from the PC to Mac server:
//   cd /cygdrive/c/Users/SANDY/Documents/Arduino/updatefw && scp * wwe@192.168.1.4:/Users/wwe/Sites/bin/firmware

#include <DueFlashStorage.h>
#include <HttpClient.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <Wire.h>
#include <JsonParser.h>
#include <Streaming.h>

#include <SPI.h>
//#include <SD.h>
#include <SdFat.h>
SdFat SD;
#define SD_CS_PIN 4

#define SERVERADDR "192.168.1.4"        // IP address (string) of Update Server (string)
#define SERVERPORT 49152                // Update Server port (integer)
#define CONFIG_PATH "/bin/config/"      // path to controller config files on Update Server (string)
#define FIRMWARE_PATH "/bin/firmware/"  // path to firmware binary files on Update Server (string)
#define BUFSIZE 4096                    // buffer size (bytes) for rcvbuf[]
#define FLASH_START_INTS ((int *)0)     // used only in commented-out code, purpose unclear

// global vars
uint8_t mac[] = {0,0,0,0,0,0};             // Ethernet adapter MAC address: used in initWeb()
char mac_chars[] = "00:00:00:00:00:00\0";  // appended to config file name: used in setup(), initWeb()
byte rcvbuf[BUFSIZE];                      // byte array buffer to hold config file contents: used in several functions
DueFlashStorage dueflashstorage;           // used in several functions


// ***We use the setup() function to do everything.***
// The loop() function is merely a placeholder until reboot occurs (see below).
void setup() {
  char cfg_filename[80];                     // configuration file name
  char binfile[40];                          // firmware binary file name
  char binpath[50];                          // firmware binary file path
  int n;                                     // retry counter
  
  Serial.begin(115200);
  Serial << "\n\n\n";
  Serial << "========\n";
  Serial << "updatefw\n";
  Serial << "========\n";

  // Initialize SD
  n = 3;
  while ( !(SD.begin(4, SPI_HALF_SPEED)) ) {
    Serial << "updatefw: setup(): SD initialization FAILED.\n";
    delay(10000);
    n--;
    if (n == 0) return;  // if we don't have the SD, we have to abort!
  }
  Serial << "updatefw: setup(): SD initialization successful.\n";

  // Initialize Ethernet
  n = 3;
  while ( !initWeb() ) {
    Serial << "updatefw: setup(): Ethernet initialization FAILED.\n";
    delay(10000);  // wait before next try
    n--;
    if (n == 0) return;  // if we don't have Ethernet, we have to abort!
  }
  Serial << "updatefw: setup(): Ethernet initialization successful.\n";
  
  // Get config file from Update Server
  sprintf(cfg_filename, "%s%s.cfg", CONFIG_PATH, mac_chars);  // MAC-specific config file name, e.g., "/config/80:1f:12:32:6c:b6.cfg"
  getConfig(SERVERADDR, SERVERPORT, cfg_filename);            // put config file contents into rcvbuf[]
  //Serial << "updatefw: setup(): " << (char*)rcvbuf << "\n";   // print contents of rcvbuf[]

  char* bin_filename = get_binary_filename();                 // get firmware binary file name from config file
  strcpy(binfile, bin_filename);                              // save name of binary file
  sprintf(binpath, "%s%s", FIRMWARE_PATH, binfile);           // save path to binary file (includes file name)
  
  // Get firmware file from HTTP server
  if (getHTTP2File(SERVERADDR, SERVERPORT, binpath, binfile)) {
    Serial << "updatefw: setup(): Firmware downloaded and verified!\n";
    
    // *** STOP HERE when uploading this program to usual Arduino flash (0x80000) for testing! ***
    // *** Meaning... comment out the rest of the code in this if() ***
    
    // NOTE: See the change to library file DueFlashStorage.h to allow an ABSOLUTE flash memory address, 
    // e.g., 0x80000 or 0xC0000. The default was an address RELATIVE to IFLASH1_ADDR = 0xC0000.
    //showNFlashBytes(40, 0x80000);  // BEFORE flash write, for debug
    writeFile2Flash(binfile, 0x80000);
    //showNFlashBytes(40, 0x80000);  // AFTER flash write, for debug

    Serial << "updatefw: GPNVM Security Bit (bit 0)     : 0 = unset, 1 = set\n";
    Serial << "updatefw: GPNVM Boot Mode (bit 1)        : 0 = boot from ROM, 1 = boot from FLASH\n";
    Serial << "updatefw: GPNVM Flash Boot Region (bit 2): 0 = FLASH0, 1 = FLASH1\n";
    Serial << "updatefw: GPNVM bits = 0b" << _BIN(getGPNVMBits(EFC0)) << "\n";
    Serial << "updatefw: Clearing GPNVM bit 2...\n";
    flash_clear_gpnvm(2);  // clear flash select bit --> on reboot, flash 0x80000 will be mapped to 0x0, and firmware binary will execute from there
    Serial << "updatefw: GPNVM bits = 0b" << _BIN(getGPNVMBits(EFC0)) << "\n";

    Serial << "updatefw: REBOOTING...\n";
    Serial.flush();

    // See https://forum.arduino.cc/t/arduino-due-software-reset/485276/5
    // It's unclear from the discussion, but there are apparently 3 alternatives here.
    // Deduced this after looking at the Atmel datasheet:
    // https://www.keil.com/dd/docs/datashts/atmel/sam3xa/doc11057.pdf - search for "software reset"
    __DSB;
    //SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);  // reboots, but SD fails to initialize
    RSTC->RSTC_CR = RSTC_CR_KEY(0xA5) | RSTC_CR_PERRST | RSTC_CR_PROCRST;           // *** THIS WORKS!!! ***
    //NVIC_SystemReset();                                                             // didn't try this
    
  }
  Serial << "updatefw: setup(): We didn't reboot to updated firmware!\n";
}
// END setup()


// Main loop() function is a dead end...
void loop() {
  delay(1);
}
// END loop()



// OTHER FUNCTIONS...

// Wrapper for readHTTP2File()
boolean getHTTP2File(char* server, uint16_t port, char* path, char* filename) {
  int n;

  n = 3;  // # retries
  //Serial << "updatefw: getHTTP2File(): Reading " << filename << "...\n";
  while (!readHTTP2File(server, port, path, filename, false)) if (n-- <= 0) return false;
  
  n = 3;  // # retries
  //Serial << "updatefw: getHTTP2File(): Verifying " << filename << "...\n";
  while (!readHTTP2File(server, port, path, filename, true)) if (n-- <= 0) return false;
  
  //Serial << "updatefw: getHTTP2File(): Download verified.\n"; 
  return(true);  // return true if we haven't return'ed false above
}


// This function retrieves a server file, possibly verifies it, and saves it to SD.
boolean readHTTP2File(char* server, uint16_t port, char* path, char* filename, boolean verify) {
  EthernetClient client;
  HttpClient http(client);
  
  byte* bufptr = rcvbuf;                // ptr to rcvbuf[]
  File thefile;                         // file ptr to SD file
  boolean returnval = false;            // return val for this function
  const int kNetworkTimeout = 30000;    // # msec to wait for data before giving up
  const int kNetworkDelay = 100;        // # msec to wait if no data is available before trying again
  boolean verify_ok = true;             // set if file verify is ok
  int bufindx = 0;                      // http method return code
  int rc = 0;                           // return code from various http functions
  int num_bytes = 0;                    // # bytes read from server
  long time_waiting = 0;                // total network wait time
  int file_index = 0;                   // file position index
  unsigned long starttime;              // total read or verify time
  unsigned long timeoutStart;           // network wait timer
  int num_errors = 0;                   // error counter
  char c;                               // a char read from the server

  // READ (no verify) a file from the server to the SD card.
  if ( !verify ) { 
    Serial << "updatefw: readHTTP2File DOWNLOADING " << server << ":" << port << path << " to SD:/" << filename << "\n";
    if (SD.exists(filename)) SD.remove(filename);  // if the file to be read exists on the SD card, delete it!

  // Otherwise... VERIFY a file from the server against an existing SD file.
  } else {
    Serial << "updatefw: readHTTP2File VERIFYING " << server << ":" << port << path << " against SD:/" << filename << "\n";
  }

  // Create a new SD file or open an existing SD file.
  if ( thefile = SD.open(filename, FILE_WRITE) ) {
    Serial << "updatefw: readHTTP2File Opened SD:/" << filename << " for write.\n";
    if ( verify ) thefile.seek(0);  // if we're verifying the file, move file ptr to the beginning of the file

    Serial << "updatefw: readHTTP2File Getting " << server << ":" << port << path << "...\n";
    starttime = millis();
    rc = http.get(server, port, path);
    http.sendHeader("Accept", "application/octet-stream");
    http.endRequest();  // REQUIRED!
    
    if ( rc == 0 ) {
      rc = http.responseStatusCode();
      //Serial << "updatefw: readHTTP2File http.responseStatusCode() = " << rc << "\n";
      if ( rc == 200 ) {
        rc = http.skipResponseHeaders();
        //Serial << "updatefw: readHTTP2File http.skipResponseHeaders() = " << rc << "\n";
        if ( rc >= 0 ) {
          int bodyLen = http.contentLength();
          Serial << "updatefw: readHTTP2File http.contentLength = " << bodyLen << " bytes\n";
          
          timeoutStart = millis();
          num_bytes = 0;
          while ( (http.connected() || http.available()) && 
                  ((millis() - timeoutStart) < kNetworkTimeout) ) {
            if ( http.available() ) {                          // if we have data available... 
              if ( verify ) {                                  //   if we're VERIFYING a file...
                c = http.read();                               //     read a char from the server
                num_bytes++;
                bodyLen--;                                     //     decrement body length by 1 char
                byte read_value = thefile.read();              //     read a char from the SD file
                if (read_value != c) {                         //     if the chars don't match...
                  verify_ok = false;                           //       set flag
                  num_errors++;                                //       increment error counter
                  if (num_errors < 100) {                      //       print the first 100 errors
                    Serial << "updatefw: readHTTP2File At addr: 0x" << _HEX(file_index) << " expected 0x" << _HEX((int)c) 
                           << " read from file: 0x" << _HEX(read_value) << "\n";
                  }
                }
                file_index++;                                  //     increment file position index
              } else {                                         //   otherwise... we're READING a file...
                int bytes_received = http.readBytes(&(rcvbuf[bufindx]), BUFSIZE - bufindx);  // read up to BUFSIZE bytes
                //Serial << "updatefw: readHTTP2File Received " << bytes_received << " bytes\n";
                bufindx += bytes_received;                     //     increment bufindx by # bytes received
                bodyLen -= bytes_received;                     //     decrement bodyLen by # bytes received
                if ((bufindx >= BUFSIZE) || (bodyLen == 0)) {  //     if we've filled the buffer or we're done reading...
                  //Serial << "updatefw: readHTTP2File Writing " << bufindx << " bytes to SD.\n";
                  thefile.write(rcvbuf, bufindx);              //       write bufindx bytes from rcvbuf to SD file
                  file_index += bufindx;                       //       increment file position index by bufindx bytes
                  num_bytes += bufindx;                        //       increment # bytes read by bufindx bytes
                  if (bodyLen == 0) break;                     //       if no more body, break out of while() loop
                  bufindx = 0;                                 //       rezero bufindx    
                }
              }  // end else reading file
              timeoutStart = millis();                         //   we read something, so rezero network timeout counter
                            
            } else {                                           // otherwise... there are no available data, so pause
              delay(kNetworkDelay);                            //   wait a bit for the network
              time_waiting += kNetworkDelay;                   //   accumulate total network wait time
            }
          }  // end while()
          
          Serial << "updatefw: readHTTP2File Downloaded " << num_bytes << " bytes\n";
          if (!verify) {
            Serial << "updatefw: readHTTP2File File read time = " << (millis() - starttime) << " msec\n";
          } else {
            Serial << "updatefw: readHTTP2File File verify time = " << (millis() - starttime) << " msec\n";
          }
          Serial << "updatefw: readHTTP2File Network wait time = " << time_waiting << " msec\n";
          
          if ( verify ) {
            if ( verify_ok ) {
              Serial << "updatefw: readHTTP2File File verified.\n";
              returnval = true;
            } else {
              Serial << "updatefw: readHTTP2File " << num_errors << " ERRORS WERE DETECTED.\n";
            }
           } else {
            returnval = true; 
          }
        } else {
          Serial << "updatefw: readHTTP2File http.skipResponseHeaders failed, rc = " << rc << "\n";
        }
      } else {
        Serial << "updatefw: readHTTP2File http.responseStatusCode failed, rc = " << rc << "\n";
      }
    } else {
      Serial << "updatefw: readHTTP2File http.get failed, rc = " << rc << "\n";
    }
    
    if ( num_bytes == 0 && !verify ) returnval = false;
    http.stop();      // stop client
    thefile.close();  // close SD file
  } else {
    Serial << "updatefw: readHTTP2File SD file open failed.\n";
  }
  return(returnval);  
}


// Write an SD file to Arduino flash memory
boolean writeFile2Flash(char* filename, int startaddr){
  boolean rc = false;
  File thefile;
  int bufaddr = 0;
  int num_bytes = 0;
  int flash_addr = startaddr;
  
  Serial << "updatefw: writeFile2Flash Flash write starting at 0x" << _HEX(startaddr) << "\n";
  if (thefile = SD.open(filename, FILE_READ)) {
    Serial << "updatefw: writeFile2Flash(): Opened SD:/" << filename << "\n";
    int bytes_remaining = thefile.size();
    while (thefile.available()) {
      rcvbuf[bufaddr++] = thefile.read();
      bytes_remaining--;
      if (bufaddr >= BUFSIZE || bytes_remaining == 0) {
        //Serial << "updatefw: writeFile2Flash(): Write " << bufaddr << " to flash at 0x" << _HEX(flash_addr) << "\n";
        dueflashstorage.write(flash_addr, rcvbuf, bufaddr);
        flash_addr += bufaddr;
        bufaddr = 0;
      }
      num_bytes++;
    }
    Serial << "updatefw: writeFile2Flash(): Wrote " << num_bytes << " bytes to flash\n";
    rc = true;
    thefile.close();
  }
  return(rc);
}


// Instantiate a JsonParser object for use in the following function
using namespace ArduinoJson::Parser;
JsonParser<80> parser;

// This function extracts the binary_filname parm from the config file.
//   If successful, the file name is returned.
char* get_binary_filename() {
  JsonObject root = parser.parse((char*)rcvbuf);
  
  if ( !root.success() ) {
    Serial << "updatefw: get_binary_filename(): JSON parsing FAILED.\n";
    Serial << "updatefw: get_binary_filename(): rcfbuf[] = " << (char*)rcvbuf << "\n";
    return(0);
  } else {
    //for (JsonObjectIterator j=root.begin(); j!=root.end(); ++j) Serial.println(j.key());
    //Serial << "updatefw: get_binary_filename(): " << (char*)root["config"]["binary_filename"] << "\n";
    return(root["config"]["binary_filename"]);
  }
  return(0);
}


int initWeb() {
  int rc = 0;  // return code
  
  Serial << "updatefw: initWeb(): Initializing Ethernet...\n";

  // Initialize I2C interface and read MAC from EEPROM.
  Wire.begin(); 
  mac[0] = readRegister(0xFA);
  mac[1] = readRegister(0xFB);
  mac[2] = readRegister(0xFC);
  mac[3] = readRegister(0xFD);
  mac[4] = readRegister(0xFE);
  mac[5] = readRegister(0xFF);
  sprintf(mac_chars, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial << "updatefw: initWeb(): MAC = " << mac_chars << "\n";

  // Initialize Ethernet
  // args = controller MAC, timeout (default 60000), response timeout (default 4000) - see Ethernet.h
  Ethernet.init(10);   // default CS pin is 10, but we set it here anyway
  if ( rc = Ethernet.begin(mac, 10000, 2000) ) {
    Serial << "updatefw: initWeb(): IP address (DHCP): " << Ethernet.localIP() << "\n";
  }
  return(rc);
}


// Used by initWeb()
byte readRegister(byte r) {
  unsigned char v;

  Wire.beginTransmission(0x50);  // 0x50 is I2C_ADDRESS
  Wire.write(r);                 // register to read
  Wire.endTransmission();
  Wire.requestFrom(0x50, 1);     // read a byte. 0x50 is I2C_ADDRESS. 
  while (!Wire.available()) {
    // Wait!
  }
  v = Wire.read();
  return v;
}



// This function calls get_config() n times.
boolean getConfig(char* server, uint16_t port, char* path) {
  int n = 3;  // # retries
  while (!get_config(server, port, path)) if (n-- <= 0) return(false);
  return(true);
}



// This function gets a config file from Update Server and put its contents into rcvbuf[] (global var).
//   This is a simplified version of getHTTP2File() found in weblclient.ino of the wwe sketch.
//   Because the config file is short, we can retrieve a series of single chars until we're done.
boolean get_config(char* server, uint16_t port, char* path) {
  EthernetClient client;
  HttpClient http(client);
  const int kNetworkTimeout = 30000;  // # msec to wait for data before giving up
  const int kNetworkDelay = 100;      // # msec to wait if no data is available before trying again
  boolean get_config_rc = true;       // return code for this function
  int http_rc = 0;                    // return code from various http functions
  int idx = 0;                        // rcvbuf[] index
  char c;                             // a char read from HTTP server
  unsigned long timeoutStart;         // network wait time counter

  Serial << "updatefw: get_config(): Getting " << server << ":" << port << path << "\n";
  http_rc = http.get(server, port, path);
  http.endRequest();  // this is REQUIRED or we get a (-) http_rc from the above line!
  
  if ( http_rc == 0 ) {
    http_rc = http.responseStatusCode();
    //Serial << "updatefw: get_config(): http.responseStatusCode = " << http_rc << "\n";
    if ( http_rc >= 0 ) {
      http_rc = http.skipResponseHeaders();
      //Serial << "updatefw: get_config(): http.skipResponseHeaders = " << http_rc << "\n";
      if ( http_rc >= 0 ) {
        int bodyLen = http.contentLength();
        //Serial << "updatefw: get_config(): http.contentLength = " << bodyLen << " bytes\n";
      
        timeoutStart = millis();
        while( (http.connected() || http.available()) && 
               ((millis() - timeoutStart) < kNetworkTimeout) ) {
          if ( http.available() ) {   // if we have data...
            c = http.read();          //   read a char
            rcvbuf[idx++] = c;        //   add it to rcvbuf[] and increment the index
            bodyLen--;                //   decrement content length
            timeoutStart = millis();  //   we read something, so reset timeoutStart
          } else {                    // otherwise...
            delay(kNetworkDelay);     //   pause to allow more data to arrive
          }
        }  // END while
      } else {
        Serial << "updatefw: get_config http.skipResponseHeaders failed, rc = " << http_rc << "\n";
        get_config_rc = false;
      }
    } else {    
      Serial << "updatefw: get_config http.responseStatusCode failed, rc = " << http_rc << "\n";
      get_config_rc = false;
    }
  } else {
    Serial << "updatefw: get_config http.get failed, rc = " << http_rc << "\n";
    get_config_rc = false;
  }
 
  http.stop();      // stop the client
  rcvbuf[idx] = 0;  // null-terminate *byte* array with a byte which has a value of 0
  
  return(get_config_rc);
  
}  // END get_config()



// Read n bytes from flash at startaddr.
void showNFlashBytes(int n, int startaddr){
  for (int i = 0; i < n; i++) {                     // for n bytes...
    int val = dueflashstorage.read(startaddr + i);  //   read a byte = 8 bits = 0x00-0xFF
    if (val < 0x10) Serial.print(0);                //   if byte val is 0x00-0x0F, print a leading 0
    Serial.print(val, 16);                          //   print hex value of the byte
    if ((i+1)%4 == 0) {                             //   if we've printed 4 bytes...
      Serial.print(" | ");                          //     print a divider
    } else {                                        //   otherwise...
      Serial.print(" ");                            //     print a space
    }
  }
  Serial.println("");                               // print a newline char
}


// Compare 2 regions in flash memory.
boolean compareFlashRegions(int n, int startaddr1, int startaddr2){
  for (int i = 0; i < n; i++) {
    if (dueflashstorage.read(startaddr1 + i) != dueflashstorage.read(startaddr2 + i)) {
      Serial << "updatefw: compareFlashRegions(): Miscompare at " << i << "\n";
      return(false);
    }
  }
  return(true);
}


// REBOOT function
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


// Get the GPNVM bits
uint32_t getGPNVMBits(Efc* p_efc) {
  // When reading gpnvm, the bit number is irrelevant.
  if (EFC_RC_OK != efc_perform_command(p_efc, EFC_FCMD_GGPB, 0)) {
    return FLASH_RC_ERROR;
  }
  return(efc_get_result(p_efc));
}
