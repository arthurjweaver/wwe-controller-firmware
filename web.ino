// ---------- web.ino ----------

#define WEBDUINO_FAIL_MESSAGE "<h1>Request Failed</h1>"
#define UDP_TX_PACKET_MAX_SIZE 200  //increase UDP packet size from 24
#define VERSION_STRING "0.1"
#define HTTP_RESPONSE_TIMEOUT 5000
#define HTTPS_RESPONSE_TIMEOUT 10000

DueFlashStorage dueflashstorage;  // instantiate a DUE flash storage object

unsigned long starttime;
char* getChannelName();
const float MAIN_SAMPLE_PERIOD_MILLIS = SAMPLE_PERIOD_MICROS / 100.0;  // = 100/100.0 = 1, period (in msec) on which all analog inputs are sampled.
const int NUM_GRAPH_CHANNELS = 11;                                     // # items in graph_channels[]
const int graph_channels[] = {0,1,2,3,9,10,11,5,6,7,8};                // 0=V1, 1=V2, 2=V3, 3=VDC, 9=V12, 10=V23, 11=V31, 5=I1, 6=I2, 7=I3, 8=IDC
const int axis_nums[] = {1,1,1,1,1,1,1,2,2,2,2};                       // y-axis numbers, used by flot

EthernetUDP statusudp;  // instantiate some Ethernet UDP client objects
EthernetUDP ntpudp;
EthernetUDP tstudp;



// This function initializes the Ethernet shield.
int initWeb() {
  uint8_t mac[] = {0,0,0,0,0,0};   // MAC address
  int ethernet_rc = 0;             // return code for Ethernet.begin()

  Serial << "web: initWeb Initializing Ethernet...\n";

  mac[0] = readEEPROMReg(0xFA);  // read decimal MAC address from EEPROM
  mac[1] = readEEPROMReg(0xFB);
  mac[2] = readEEPROMReg(0xFC);
  mac[3] = readEEPROMReg(0xFD);
  mac[4] = readEEPROMReg(0xFE);
  mac[5] = readEEPROMReg(0xFF);
  sprintf(mac_chars, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);  // create MAC hex string
  Serial << "web: initWeb MAC (hex) = " << mac_chars << "\n";
  
  String controllerIP = String(parm_controller_ip.parmVal());  // convert controller IP to type String, so we can...
  controllerIP.toUpperCase();                                  // convert it .toUpperCase(), so we can...

  if ( controllerIP != "DHCP" ) {                              // assign static IP...
    Ethernet.begin( mac, parm_controller_ip.IPVal(), parm_dns1_ip.IPVal(), parm_gateway_ip.IPVal() );  // this function variant returns void!
    ethernet_rc = 1;                                           //   so, assume success and set return code to 1
    Serial << "web: initWeb Using Static IP " << Ethernet.localIP() << "\n";
  } else {                                                     // or, use DHCP...
    ethernet_rc = Ethernet.begin(mac, 10000, 2000);            //   timeout default = 60000, response timeout default = 4000 (msec) - see Ethernet.h
                                                               //   this function variant returns 0 if DHCP config fails, 1 if it succeeds
    Serial << "web: initWeb DHCP-assigned IP " << Ethernet.localIP() << "\n";
    Serial << "web: initWeb DHCP-assigned GW " << Ethernet.gatewayIP() << "\n";
    Serial << "web: initWeb DHCP-assigned DNS " << Ethernet.dnsServerIP() << "\n";
  }
  
  if ( ethernet_rc ) { 
    Serial << "web: initWeb Ethernet initialization successful.\n";

    Serial << "web: initWeb Getting NTP time...\n";
    //sendUDPWorkaround();                          // this may be necessary even with Ethernet 2 shield - see function below
    unsigned long ntptime = ntpUnixTime(ntpudp);  // get NTP time (unix time format), returns 0 if it fails

    // Set the PROGRAM time (myunixtime)
    Serial << "web: initWeb Getting RTC time...\n";
    rtc_time = realtime_clock.get();                                                   // get RTC time (unix time format)
    if ( rtc_time ) {                                                                  // if we have RTC time...
      Serial << "web: initWeb RTC time = " << rtc_time << "\n";                        //   print RTC time
      if ( ntptime ) {                                                                 //   if we have NTP time...
        myunixtime = ntptime;                                                          //     ***set PROGRAM time to NTP***
        Serial << "web: initWeb NTP time = " << ntptime << "\n";                       //     print NTP time
        Serial << "web: initWeb Adjusting RTC by " << ntptime - rtc_time << " sec\n";  //     print NTP-RTC time diff
        realtime_clock.set(ntptime);                                                   //     adjust RTC to NTP time
        Serial << "web: initWeb RTC time = " << rtc_time << "\n";                      //     print adjusted RTC time
      } else {                                                                         //   if we DON'T have NTP time...
        myunixtime = rtc_time;                                                         //     ***set PROGRAM time to RTC***
      }
    } else {                                                                           // if we DON'T have RTC time...
      Serial << "web: initWeb get RTC time FAILED.\n";                                 //   print FAIL message
      if ( ntptime ) {                                                                 //   if we have NTP time...
        myunixtime = ntptime;                                                          //     ***set PROGRAM time to NTP***
        Serial << "web: initWeb NTP time = " << ntptime << "\n";                       //     print NTP time
      } else {                                                                         //   if we DON'T have NTP time...
        Serial << "web: initWeb get NTP time FAILED.\n";                               //     print FAIL message
        while(1) {                                                                     //     DEAD END
          Serial << "web: initWeb NO RTC or NTP available. PROGRAM STOPPED.\n";
          delay(3600000);  // 1 hour
        }
      }
    }

    // Start a webserver - see webserver.ino
    initServer();

#ifdef USE_TEST_VALS
    tstudp.begin(udp_rcv_port);
#endif

  } else {
    Serial.println("web: initWeb Ethernet initialization FAILED.");
  }
  return( ethernet_rc );
}




// This function is a hack to kickstart Ethernet UDP before we use it for other UDP stuff.
//   It broadcasts a dummy UDP packet to the local network (255.255.255.255)
//   Tests suggest it is still needed with the Ethernet Shield 2.
// Regarding broadcast address - see https://en.wikipedia.org/wiki/Broadcast_address#IP_networking
//   "A special definition exists for the IP address 255.255.255.255. It is the broadcast address of 
//   the zero network or 0.0.0.0, which in Internet Protocol standards stands for this network, i.e. 
//   the local network. Transmission to this address is limited by definition, in that it is never 
//   forwarded by the routers connecting the local network to other networks."
void sendUDPWorkaround() {
  statusudp.begin(555);                         // start a UDP client, listening on an arbitrary port
  static uint8_t udp_ip[] = {255,255,255,255};  // broadcast to the local network
  statusudp.beginPacket(udp_ip, 55555);         // open packet for sending to (server or broadcast, port)
  statusudp.write("wwe");                       // write some random chars
  statusudp.endPacket();                        // close the packet
  statusudp.stop();                             // stop UDP client
}




// Send a UDP packet with current parms to the UDP server.
int sendConfigUDP(char* ip_str) {
  int rc = 0;                          // return code
  statusudp.begin(456);                // start a UDP client, listening on an arbitrary port
  unsigned long starttime = millis();  // measure how long this takes

  Serial << "web: sendConfigUDP() Sending current parms to UDP server: " << ip_str << ":" << udp_remote_port_config << "\n";
  statusudp.beginPacket(ip_str, udp_remote_port_config);  //
  statusudp.write(jsonbuf);
  statusudp.endPacket();

  // Wait for a response from the UDP server.
  const int pollIntv = 100;                                // check every 100 msec
  const byte maxPoll = 10;                                 // up to 10 times
  int pktLen;                                              // received packet length
  for (byte i=0; i < maxPoll; i++) {                       // start polling loop...
    if ( (pktLen = statusudp.parsePacket()) != 0 ) break;  //   if we get a response, break out of for() loop
    delay(pollIntv);                                       //   wait
  }
  // Handle response from UDP server (if any).
  if (pktLen > 0) {                                        // if we get a response from the UDP server...
    Serial << "web: sendConfigUDP() Received " << pktLen << " bytes from UDP server:\n";
    statusudp.read(jsonbuf, pktLen);                       //   read response into jsonbuf
    jsonbuf[pktLen] = 0;                                   //   null terminate jsonbuf string
    printJSONBuf();                                        //   print jsonbuf to Serial
    rc = writeBuff2Parms();                                //   put changes (if any) into the controller parms, see parms.ino
  } else {                                                 // otherwise, we have no response...
    rc = 0;                                                //   set return code = 0
    Serial << "web: sendConfigUDP() NO RESPONSE from UDP server.\n";
  }  
  statusudp.stop();                   // stop
  Serial << "web: sendConfigUDP() cycle time = " << (millis()-starttime) << " msec\n";

  return(rc);
}



// This function calls the next one, of the same name, after converting the UDP IP string to an array.
void sendDataUDP(char* ip_str, int udp_remote_port, int do_modbus) {
  static uint8_t udp_ip[] = {255, 255, 255, 255};   // int array
  static char last_ip_str[20] = "255.255.255.255";  // string (char array)

  // Convert destination UDP IP string to an array
  //   strtok() usage - see https://www.tutorialspoint.com/c_standard_library/c_function_strtok.htm
  if (strcmp(ip_str, last_ip_str) != 0) {  // if the incoming IP string and last_ip_str don't match...
    int i = 0;                             //   token counter
    char* token;                           //   string pointer returned by strtok()
    strcpy(last_ip_str, ip_str);           //   copy ip_str --> last_ip_str
    token = strtok(last_ip_str, ".");      //   break last_ip_str into tokens separated by delimiter "."
    while (token != NULL) {                //   loop through the tokens...
      udp_ip[i++] = atoi(token);           //     each (char*) token becomes an (int) element of udp_ip[]
      token = strtok(NULL, ".");           //     advance pointer to next token
    }
    strcpy(last_ip_str, ip_str);           //   copy ip_str --> last_ip_str, because last_ip_str was destroyed by strtok()
  }
  //Serial << "web: sendDataUDP " << ip_str << "--> " << udp_ip[0] << "." << udp_ip[1] << "." << udp_ip[2] << "." <<  udp_ip[3] << "\n";
  sendDataUDP(udp_ip, udp_remote_port, do_modbus);  // call same-named function (below) with IP array arg
}



// Send out a JSON string as a UDP packet.
// This function calls printPOSTBody() in webclient.ino to construct the actual UDP packet.
//   uint8_t* is a pointer to an array 8-bit ints holding the UDP IP address
void sendDataUDP (uint8_t* udp_ip, int udp_remote_port, int do_modbus) {
  unsigned long starttime = millis();
  
  //Serial << "web: calling statusudp.begin()...\n";
  statusudp.begin(456);  // start a UDP client, listening on an arbitrary port

  //Serial << "web: calling statusudp.beginPacket()...\n";
  statusudp.beginPacket(udp_ip, udp_remote_port);

  //Serial << "web: calling printPOSTBody()...\n";
  printPOSTBody(true, false, do_modbus);  // args: do_udp, countonly, do_modbus (0, 1, 2, or 3)

  //Serial << "web: calling statusudp.endPacket()...\n";
  statusudp.endPacket();
  
  //Serial << "web: calling statusudp.stop()...\n";
  statusudp.stop();
  
  unsigned long udp_duration = millis() - starttime;

  Serial << "web: sendDataUDP --> " << udp_ip[0] << "." << udp_ip[1] << "." << udp_ip[2] << "." << udp_ip[3] << ":" 
                                   << udp_remote_port << ", send time = " << udp_duration << " msec\n";
}


// Left over from early testing...
#ifdef USE_TEST_VALS
using namespace ArduinoJson::Parser;
// Implement a server to handle test-mode inputs from incoming UDP packets.
void getTestInputs() {

  static boolean firsttime = true;         // true only on first call to this function
  double T, F, I, V, Vw, Tf;               // unixtime, frequency, current, voltage, ???, furl duration
  char json_strg[UDP_TX_PACKET_MAX_SIZE];  // set JSON string length to max UDP packet size
  JsonParser<20> parser;

  //char json_strg[] = "{\"T\": \"07/08/14 20:11:23\", \"F\": 34.0, \"I\": 11.0, \"V\": 277.0, \"Vw\": 19.0}";  // DEBUG
  //tstudp.begin(udp_rcv_port);

  int packet_len = tstudp.parsePacket();  // parse a test UDP packet, returns packet length
  if ( tstudp.available() ) {
    
    Serial << "web: getTestInputs Received test UDP packet, length = " << packet_len << "\n";
    json_strg[packet_len] = 0;  // Not sure why I need to null terminate this, but I seem to.

    Serial << "web: getTestInputs Reading test UDP packet...\n";
    tstudp.read(json_strg, UDP_TX_PACKET_MAX_SIZE);      // read up to UDP_TX_PACKET_MAX_SIZE chars from tstudp into json_strg

    Serial << "web: getTestInputs Parsing JSON...\n";  
    JsonObject root = parser.parse(json_strg);           // parse json_strg into a JSON object (root)

    if ( root.success() ) {
      Serial << "web: getTestInputs Setting test vals...\n";
      T = root["T"]; unixtime = T;
      F = root["F"]; setTestValue(12, F);
      I = root["I"]; setTestValue(5, I); setTestValue(6, I); setTestValue(7, I); setTestValue(8, I);
      V = root["V"]; setTestValue(0, V); setTestValue(1, V); setTestValue(2, V); setTestValue(3, V);
                     setTestValue(9, V); setTestValue(10, V); setTestValue(11, V);
      Vw = root["Vw"]; setTestValue(13, Vw);

      if ( firsttime ) {
        firsttime = false;
        Serial << "web: getTestInputs Setting furl duration from test file!: \n";
        Tf = root["Tf"]; furl_duration = (int)Tf;
        Serial << Tf << " " << furl_duration << "\n";
      }
      
      threshold_checkers[0].check() ? Serial << "web: getTestInputs Voltage is HIGH.\n" : Serial << "web: getTestInputs Voltage is OK.\n";
      Serial << "web: getTestInputs RMS Voltage = " << getChannelRMSInt(L1L2_VOLTAGE) << "\n";

      // Now that we have the vals, call furlctl() to act on them.
      //Serial << "web: getTestInputs Calling furlctl()...\n";
      //furlctl();
      
    } else {  // END if ( root.success() )
      Serial << "web: getTestInputs JSON parse FAILED: \n";
      Serial << json_strg << "\n";
    }
  }
  //tstudp.stop();
}
#endif
