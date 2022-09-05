// ---------- webclient.ino ----------
// HTTP and HTTPS GET and POST functions to send and retrieve data to/from a server.

// used in readHttp2File() and writeFileToFlash()
#define BUFSIZE 4096

// Create an HTTP client
EthernetClient client;
HttpClient http(client);  // see methods in <HttpClient.h>

// Create an HTTPS client
// An SSL certificate MUST be generated that includes the server(s) to be visited, e.g., api.weather.gov
//   see https://openslab-osu.github.io/bearssl-certificate-utility/
//   and put the generated certificate into the #include'd file "trust_anchors.h"
#include "trust_anchors.h"
// Choose an analog input pin to get semi-random data from for SSL. 
// ***Pick a pin that's either not connected or attached to a randomish voltage source.***
// OBSERVATION: The HTTPS code was HANGING after anywhere from one to a few hundred API calls! 
//   The problem was narrowed down to the analogRead() statements in SSLClient::m_start_ssl.
// CONJECTURE: Analog pin A5 is Current Sense Vref which is being read at 1000 Hz in adc.ino! 
//   The analogRead(A5) statements in SSLClient::m_start_ssl fail or somehow conflict with this high
//   read rate. Regardless, usage notes say to use an UNCONNECTED analog input pin, which A5 is NOT!
// OBSERVATION: Changing from A5 to A6 did NOT fix the problem. So, SSLClient.cpp was MODIFIED:
//   //rng_seeds[i] = static_cast<uint8_t>(analogRead(m_analog_pin));  // this periodically FAILS and HANGS the code!
//   rng_seeds[i] = static_cast<uint8_t>(random(256));  // generate pseudo-random 8-bit numbers instead
//
// The last arg to https() is the debug output level: SSL_NONE, SSL_ERROR, SSL_WARN, SSL_INFO, or SSL_DUMP
//   see https://github.com/OPEnSLab-OSU/SSLClient, scroll down to Logging
EthernetClient base_client;
SSLClient https(base_client, TAs, (size_t)TAs_NUM, A6, 1, SSLClient::SSL_ERROR);



// This function does a GET request *and* parses data received from the NWS API ***HTTPS*** server.
//   It is called by getWeatherData - see weather.ino.
// We parse data on-the-fly to avoid storing a large amount of unneeded data in a long JSON string
//   before processing it. Why? Arduino memory is limited!
// An SSL certificate MUST be generated that includes the host domain: api.weather.gov
//   see https://openslab-osu.github.io/bearssl-certificate-utility/
//   and put the generated certificate into the #include'd file "trust_anchors.h"
void getNWSAPIData(char* theserver, char* thepath) {
  int bytecount = 0;
  static boolean wxalert = false;  // weather alert flag, static preserves its last state

  Serial << "webclient: getNWSAPIData server = " << theserver << "\n";
  Serial << "webclient: getNWSAPIData path = " << thepath << "\n";

  // SSLClient says initial .connect() SSL negotiation can take 4-10 seconds!
  // We see ~4 seconds on the FIRST API call, then ~400 msec on subsequent calls.
  https.setTimeout(20000);  // default is 30000 ms

  // For SSLClient class, see SSLClient.cpp
  // For EthernetClient class, see EthernetClient.cpp
  // PROGRAM FLOW:
  //   https.connect() == SSLClient::connect --> EthernetClient::connect --> (back to) SSLClient::connect --> 
  //   SSLClient::m_start_ssl --> SSLClient::m_run_until --> (back to) SSLClient::m_start_ssl --> https.connect() returns 1
  //
  auto connecttime = millis();
  //if ( https.connect(theserver, 443) ) { 
  IPAddress ipaddr = {23,33,56,21};         // <-- nslookup api.weather.gov, avoids DNS
  if ( https.connect(ipaddr, 443) ) {       // port 443 = HTTPS
    Serial << "webclient: getNWSAPIData connect time = " << (millis() - connecttime) << " msec\n";
    Serial.flush();
    https.print("GET ");
    https.print(thepath);      // path name MUST begin with "/"
    https.println(" HTTP/1.1");
    https.println("User-Agent: SSLClientOverEthernet");
    https.print("Host: ");
    https.println(theserver);  // server name MUST match a domain *name* listed in trust_anchors.h
    https.println("Connection: close");
    https.println();
  } else {
    https.stop();  // we MUST stop the client or subsequent Ethernet access is blocked!
    Serial << "webclient: getNWSAPIData connection failed.\n";
    return;
  }

  // Skip over headers and anything else until we get to the first "{" char of the JSON data,
  //   then break out of the while() and move on. (There are no methods in SSLClient to do this.)
  auto waittime = millis();
  while ( true ) {
    if ( https.available() ) {
      char c = https.read();
      if ( c == '{' ) break;  // '{' is a char, "{" would be a (null-terminated) string which would fail test
    }
    if ( (millis() - waittime) > 10000 ) {
      https.stop();  // MUST stop the client or subsequent Ethernet access is blocked!
      Serial << "webclient: getNWSAPIData timed out.\n";
      return;
    }
  }

  // Read through JSON data one char at a time
  // Look for any occurances of "wind" or "Wind" or "WIND" in the Alert/Warning/Advisory
  while ( true ) {
    if ( https.available() ) {
      char cbuf[] = "abcd";                                             // initialize string, includes \0 at end --> necessary for printing it!
      int len = https.read((byte*)&cbuf[0], 1);                         // read one byte, char ptr --> byte ptr
      if ( len == 1 ) bytecount++;                                      // increment byte counter
      cbuf[0] = toupper( cbuf[0] );                                     // convert char to uppercase
      if ( cbuf[0] == 'W' ) {                                           // if we've found a 'w' or 'W'...
        len = https.read((byte*)&cbuf[1], 3);                           //   read next 3 bytes, char ptr --> byte ptr
        bytecount += len;                                               //   increment byte counter by #bytes read
        if ( len == 3 ) {                                               //     if we have 3 more bytes (chars)...
          for (int i = 1; i < 4; i++) cbuf[i] = toupper( cbuf[i] );     //       convert them to uppercase
          Serial << "webclient: getNWSAPIData buf = " << cbuf << "\n";  //       print cbuf
          if ( strcmp(cbuf, "WIND") == 0 ) wxalert = true;              //       if we find the string we're looking for, set weather alert flag
        }
      }
    }
    
    if ( !https.connected() ) {   // when we're disconnected (by the host)...
      https.stop();               //   stop client
      break;                      //   escape while()
    }
  }

  wxalert ? weather_furl = true : weather_furl = false;  // set weather FURL flag (global), used in furlctl.ino
  Serial << "webclient: getNWSAPIData data length = " << bytecount << " bytes\n";
  return;           // return the data
  
}  // END getNWSAPIData



// This function does an ***HTTPS*** GET request.
//   see https://github.com/OPEnSLab-OSU/SSLClient/blob/master/examples/EthernetHTTPS/EthernetHTTPS.ino
// You MUST generate an SSL certificate that includes all of the HTTPS request host domains:
//   see https://openslab-osu.github.io/bearssl-certificate-utility/
//   and put the generated certificate into the #include'd file "trust_anchors.h"
char* doHttpsGet(char* server_addr, char* filename) {

  Serial << "webclient: doHttpsGet server = " << server_addr << "\n";
  Serial << "webclient: doHttpsGet path = " << filename << "\n";

  // SSLClient says .connect() should take 4-10 seconds!
  https.setTimeout(20000);  // if not set, default is 30000 ms 

  auto starttime = millis();
  if ( https.connect(server_addr, 443) ) {  // port 443 = HTTPS
    auto connecttime = millis() - starttime;
    Serial << "webclient: doHttpsGet connected in = " << connecttime << " msec\n";
    https.print("GET ");
    https.print(filename);  // must begin with "/"
    https.println(" HTTP/1.1");
    https.println("User-Agent: SSLClientOverEthernet");
    https.print("Host: ");
    https.println(server_addr);  // this MUST match a domain listed in trust_anchors.h!
    https.println("Connection: close");
    https.println();
  } else {
    https.stop();  // we MUST stop the client or subsequent Ethernet access is blocked!
    Serial << "webclient: doHttpsGet connection failed.\n";
    return("");
  }
  return( getHttpsResponse() );  // return the server response

}  // END doHttpsGet()



// CURRENTLY UNUSED.
// This function does an ***HTTPS*** POST request.
//   see comments for doHttpPost
char* doHttpsPost(char* server_addr, char* filename, char* postdata) {

  auto starttime = millis();
  if ( https.connect(server_addr, 443) ) {  // port 443 = HTTPS
    auto connecttime = millis() - starttime;
    Serial << "webclient: doHttpsPost https.connect = " << connecttime << " msec\n";
    https.print("POST ");
    https.print(filename);  // must begin with "/"
    https.println(" HTTP/1.1");
    https.println("User-Agent: SSLClientOverEthernet");
    https.print("Host: ");
    https.println(server_addr);  // this MUST match a domain listed in trust_anchors.h!
    https.println("Connection: close");
    https.println("Content-Type: application/json");
    https.print("Content-Length: ");
    https.println( strlen(postdata) );  // tell server how much data we're posting!
    https.println();
    https.print(postdata);
  } else {
    https.stop();  // MUST stop the client or subsequent Ethernet access is blocked!
    Serial << "webclient: doHttpsPost connection failed.\n";
  }
  return( getHttpsResponse() );  // return the server response
  
}  // END doHttpsPost()



// CURRENTLY UNUSED.
// This function does an ***HTTP*** GET request.
char* doHttpGet(char* server_addr, char* filename) {

  if ( client.connect(server_addr, 80) ) {
    client.print("GET /");
    client.print(filename);
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(server_addr);
    client.println("Connection: close");
    client.println();
  }
  return( getHttpResponse() );  // return the server response
  
} // END doHttpGet



// CURRENTLY UNUSED and UNTESTED - see test code in wwe.ino
// This function does an ***HTTP*** POST request.
// Every second, the controller posts a JSON string (filename) to server_addr:server_port containing current system operating data.
char* doHttpPost(char* server_addr, int server_port, char* filename) {

  if ( client.connect(server_addr, server_port) ) {
    Serial << "webclient: doHttpPost connected.\n";
    client.print("POST ");
    client.print(filename);
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(server_addr);
    client.println("Connection: close");
    client.println("Content-Type: application/json");
    
    client.print("Content-Length: ");
    client.println( printPOSTBody(false, true, 0) );  // do_udp==false, countonly==true, 0==controller data, returns content length
    client.println("");
    printPOSTBody(false, false, 0);  // do_udp==false, countonly==false, 0==controller data, sends JSON-formatted data to server

    return( getHttpResponse() );
  } else {
    Serial << "webclient: doHttpPost connect FAILED.\n";
  }
  
}  // END doHttpPost



// This function returns a controller or Modbus channel name. Used by printPOSTBody() immediately below.
// There is a method of the same name in class ModbusReg{} - see modbus.h
char* getChanName(int i, int do_modbus) {
  if ( do_modbus == 0 ) {
    return( getChannelName(i) );  // see adc.ino
  } else {
    return( getModchannelName(do_modbus, i) );  // see modbus.h
  }
}



// This function generates (in short chunks) a JSON string for posting.
// This function calls printOrCount() immediately below.
int printPOSTBody(boolean do_udp, boolean countonly, int do_modbus) {
  int num_channels;  // # channels (or data vals)
  char buf[40];      // string buffer
  char ipstr[16];    // IP address string
  int len = 0;       // string length
  
  if (do_modbus == 0) num_channels = NUM_ADC_CHANNELS;       // 0 --> Controller data, NUM_ADC_CHANNELS is defined in adc.ino
  if (do_modbus == 1) num_channels = NUM_MOD_FAST_CHANNELS;  // 1 --> Morningstar Modbus 'fast' regs, NUM_MOD_FAST_CHANNELS is defined in modbus.h
  if (do_modbus == 2) num_channels = NUM_MOD_SLOW_CHANNELS;  // 2 --> Morningstar Modbus 'slow' regs, NUM_MOD_SLOW_CHANNELS is defined in modbus.h
  if (do_modbus == 3) num_channels = NUM_MOD_NUV_CHANNELS;   // 3 --> Nuvation Modbus/TCP regs, NUM_MOD_NUV_CHANNELS is defined in modbus.h

  // Begin JSON with "id" and "ip" key:vals
  len += printOrCount(do_udp, countonly, "{\"id\":\"");                         // print {"id":"
  len += printOrCount(do_udp, countonly, mac_chars);                            // print hex-coded MAC address
  
  len += printOrCount(do_udp, countonly, "\",\"ip\":\"");                       // print ","ip":"
  IPAddress ipaddr = Ethernet.localIP();                                        // get IP address (as int array)
  sprintf(ipstr, "%d.%d.%d.%d", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);    // convert IP address to string
  len += printOrCount(do_udp, countonly, ipstr);                                // print IP string

  // CHANNELS
  len += printOrCount(do_udp, countonly, "\",\"channels\":[\"");                // print ","channels":["
  for (int i = 0; i < num_channels; i++) {                                      // loop over channel names...
    if (getChanName(i, do_modbus) == "ABloD60") continue;                       //   skip to next channel name
    if (getChanName(i, do_modbus) == "ABloD2") continue;                        //   skip to next channel name
    if (getChanName(i, do_modbus) == "ABhiD60") {                               //   if channel name is ABhiD60...
      printOrCount(do_udp, countonly, "ABD60\",\"");                            //     rename it (with trailing quote followed by comma)
      continue;                                                                 //     skip to next channel name
    }
    if (getChanName(i, do_modbus) == "ABhiD2") {                                //   if channel name is ABhiD60...
      printOrCount(do_udp, countonly, "ABD2\",\"");                             //     rename it (with trailing quote followed by comma)
      continue;                                                                 //     skip to next channel name
    }
    len += printOrCount(do_udp, countonly, getChanName(i, do_modbus));          //   print channel name
    if (i < num_channels - 1) len += printOrCount(do_udp, countonly, "\",\"");  //   print "," between channel names
  }                                                                             // END loop over channel names

  // DATA
  len += printOrCount(do_udp, countonly, "\"],\"data\":[");               // print "],"data":[
  sprintf(buf, "{\"time\":%d,\"vals\":[", myunixtime);                    // put timestamp into buf
  len += printOrCount(do_udp, countonly, buf);                            // print buf {"time":<timestamp>,"vals":[

  int ABD60 = 0, ABD2 = 0;                                                // Alarm vals (low word + high word)
  for ( int i = 0; i < num_channels; i++ ) {                              // loop over channel vals...
    if (do_modbus) {                                                      //   if MODBUS data...
      sprintf(buf, "%s", getModchannelValue(do_modbus, i));               //     put val into string buf
 
      if (getChanName(i, do_modbus) == "ABloD60") {                       //     if the channel name is a low word...
        ABD60 = mod_fast_regs[i]->valInt();                               //       save its val (as int!)
        continue;                                                         //       skip to next channel
      }
      if (getChanName(i, do_modbus) == "ABhiD60") {                       //     if the channel name is a high word...
        ABD60 += (mod_fast_regs[i]->valInt() << 16);                      //       add high word to low word 
        if ( strcmp(buf, "\"NaN\"") != 0 ) sprintf(buf, "%d", ABD60);     //       save combined val in buf (if not "NaN")
      }
      if (getChanName(i, do_modbus) == "ABloD2") {                        //     if channel name is a low word...
        ABD2 = mod_fast_regs[i]->valInt();                                //       save its val (as int!)
        continue;                                                         //       skip to next channel
      }
      if (getChanName(i, do_modbus) == "ABhiD2") {                        //     if the channel name is a high word...
        ABD2 += (mod_fast_regs[i]->valInt() << 16);                       //       add hi word to low word 
        if ( strcmp(buf, "\"NaN\"") != 0 ) sprintf(buf, "%d", ABD2);      //       save combined val in buf (if not "NaN")
      }
    } else {                                                              //   otherwise, CONTROLLER data...
      float multiplier = 1.0;
      if (i == TAIL_POSITION) {                                                       // TAIL_POSITION
        multiplier = 0.002903226;                                                     //   units are usteps, with 360/(2000*62) deg/ustep --> multiplier = 0.002903226
        sprintf(buf, "%.2f", (float)getChannelRMSInt(TAIL_POSITION) * multiplier);    //   get val and format (as float)
      } else if (i == HVDL_DUTY_CYCLE) {                                              // HVDL_DUTY_CYCLE
        multiplier = 0.01;                                                            //   units are % --> multiplier = 0.01
        sprintf(buf, "%.2f", (float)getChannelRMSInt(HVDL_DUTY_CYCLE) * multiplier);  //   get val and format (as float)
      } else if (i == STATE) {                                                        // STATE, no units (bitfield) --> no multiplier
        sprintf(buf, "%d", getChannelRMSInt(STATE));                                  //   get val and format (as int)
      } else {                                                                        // ALL OTHER CHANNELS...
        multiplier = 0.0009765625;                                                    //   vals are 1024x actual --> multiplier = 1/1024
        sprintf(buf, "%.2f", (float)getChannelRMSInt(i) * multiplier);                //   get val and format (as float)
      }
      if (strncmp(buf, "inf", 3) == 0) strcpy(buf, "\"inf\"");      // if val == "inf", insert "inf" into buf
    }
    len += printOrCount(do_udp, countonly, buf);                    // print buf (contains channel val)
    i < num_channels - 1 ? sprintf(buf, ",") : sprintf(buf, "]}");  // print , (between vals) or ]} (after last val) to buf
    len += printOrCount(do_udp, countonly, buf);                    // print buf
  }  // END loop over channel vals

  len += printOrCount(do_udp, countonly, "]}");  // end "data" array with ], end JSON string with }
  //Serial << "webclient: printPOSTBody string length = " << len << "\n";
  return(len);  
}



// This function is called by printPOSTBody().
//   If countonly==true, just calculate the length of strg and return it.
//   If countonly==false AND do_udp==true, write a UDP packet containing strg to a UDP client (statusudp).
//   If countonly==false AND do_udp==false, write strg to an HTTP client.
int printOrCount(boolean do_udp, boolean countonly, char* strg) {
  
  if (countonly) {          // if countonly==true...
    return(strlen(strg));   //   return string length
  } else if (do_udp) {      // otherwise, countonly==false, do_udp==true
    //Serial << strg;         //   display string on serial monitor, and
    statusudp.write(strg);  //   ***write string to UDP client***
    return(strlen(strg));   //   return string length
  } else {                  // otherwise, countonly==false and do_udp==false
    client.write(strg);     //   ***write string to HTTP client***
    return(0);              //   return 0
  }
}



// This function gets a response from an ***HTTP*** server after a GET or POST.
char* getHttpResponse() {
  int bytecount = 0;
  strcpy(jsonbuf, "");  // global var, see wwe.ino

  unsigned long starttime = millis();
  while ( true ) {
    unsigned long currenttime = millis();
    if ( (currenttime - starttime) > HTTP_RESPONSE_TIMEOUT ) {
      client.stop();
      Serial << "webclient: getHttpResponse timed out.\n";
      break;  // escape while()
    }
    // Read server data into a small buffer.
    // This is more efficient than reading (and printing) one char at a time!
    int len = client.available();   // len == # chars available
    if ( len > 0 ) {                // if we have chars...
      byte buf[100];                //   create a buffer for them
      if (len > 100) len = 100;     //   don't overflow the buffer!
      client.read(buf, len);        //   read len chars into buffer
      Serial.write(buf, len);       //   print len chars from buffer to serial monitor (DEBUG)
      strcat(jsonbuf, (char*)buf);  //   add buf to jsonbuf
    }
    bytecount += len;
    if ( !client.connected() ) {
      client.stop();
      break;  // escape while()
    }
  }
  strcat(jsonbuf, "0");      // add a "0" char to the end of the string... DO WE NEED THIS? WHERE IS IT USED?
  Serial << "webclient: getHttpResponse data length = " << bytecount << " bytes\n";
  return(jsonbuf);           // return the data

}  // END getHttpResponse



// This function gets a response from an ***HTTPS*** server after a GET or POST.
// The intended response should be {}-delmited JSON data.
char* getHttpsResponse() {
  int bytecount = 0;

  // Skip over headers and anything else until we get to the first "{" char of the JSON data,
  //   then break out of the while() and move on. (There are no methods in SSLClient to do this.)
  auto waittime = millis();
  while ( true ) {
    if ( https.available() ) {
      char c = https.read();
      if ( c == '{' ) break;  // '{' is a char, "{" would be a (null-terminated) string which would fail test
    }
    if ( (millis() - waittime) > 10000 ) {
      https.stop();  // MUST stop the client or subsequent Ethernet access is blocked!
      Serial << "webclient: getHttpsResponse timed out.\n";
      return("");
    }
  }
  Serial.write("{");

  // Read JSON data into a small buffer.
  // This is more efficient than reading (and printing) one char at a time!
  while ( true ) {
    int len = https.available();   // len == # chars available
    if ( len > 0 ) {               // if we have chars...
      byte buf[100];               //   create a buffer for them
      if (len > 100) len = 100;    //   don't overflow the buffer!
      https.read(buf, len);        //   read len chars into buffer
      Serial.write(buf, len);      //   print len chars from buffer to serial monitor (DEBUG)
    }
    bytecount += len;
    
    if ( !https.connected() ) {   // if we disconnect...
      https.stop();               //   stop client
      Serial.println();           //   print newline following disconnect
      break;                      //   escape while()
    }
  }
  Serial << "webclient: getHttpsResponse data length = " << bytecount << " bytes\n";
  return("");           // return the data
  
}  // END getHttpsResponse






// CURRENTLY UNUSED and UNTESTED.
// This function does an ***HTTP*** POST request using methods in the HttpClient lib
void doPOSTString(char* serveraddr, int serverport, char* filename, char* postbody, char* rcvbuf) {
  
  int rc = 0;
  char* bufptr = &postbody[0];
  int bufindex = 0;
  const int kNetworkTimeout = 5*1000;  // msec to wait w/o receiving data before giving up
  const int kNetworkDelay = 1000;      // msec to wait w/o available before trying again

  rc = http.post(serveraddr, serverport, filename);
  http.sendHeader( HTTP_HEADER_CONTENT_LENGTH, strlen(postbody) );
  http.sendHeader( HTTP_HEADER_CONNECTION, "close" );
  http.endRequest();
  http.print(postbody);
  
  if (rc == 0){
    Serial << "webclient: doPOSTString Request OK, waiting for response...\n";

    rc = http.responseStatusCode();
    if ( rc == 200 ) {
      Serial << "Return status code: 200\n";
      rc = http.skipResponseHeaders();
      
      if (rc >= 0) {
        int bodyLen = http.contentLength();
        Serial << "Content length is: " << bodyLen << "\n";
        Serial << "Body returned follows:\n";

        unsigned long timeoutStart = millis();
        while ( ( http.connected() || http.available() ) &&
                ( (millis() - timeoutStart) < kNetworkTimeout ) ) {
          if ( http.available() ) {
            bufindex += http.readBytes( &rcvbuf[bufindex], bodyLen - bufindex );
            if ( bufindex == bodyLen ) {
              rcvbuf[bufindex] = 0;
              Serial << rcvbuf << "\n";
              break;
            }
            timeoutStart = millis();
          } else {
            delay(kNetworkDelay);
          }
        }
      } else {
        Serial << "webclient: doPOSTString Failed to skip response headers: rc = " << rc << "\n";
      }
    } else {    
      Serial << "webclient: doPOSTString Getting response failed: rc = " << rc << "\n";
    }
  } else {
    Serial << "webclient; doPOSTString Connect failed: rc = " << rc << "\n";
  }
  Serial << "webclient: doPOSTString Stopping HTTP client...\n";
  http.stop();
  
}  // END doPOSTString



// This function calls readHttp2File() which gets a file from the web and saves it to SD.
// ***IMPORTANTLY, it gets the bootloader sketch ("fwupdate.bin") that updates firmware***
boolean getHttp2File(char* server, uint16_t port, char* path, char* filename) {

  int n = 3;  // try 3 times...
  while (!readHttp2File(server, port, path, filename, false)) {  // false --> just download file, don't verify
    if (n-- <= 0) return false;
  }
  n = 3;  // try 3 times...
  while (!readHttp2File(server, port, path, filename, true)) {  // true --> verify downloaded file against existing file
    if (n-- <= 0) return false;
  }
  Serial << "webclient: getHttp2File: Download verified.\n";
  return(true);
}


// This function does an ***HTTP*** GET request and EITHER:
//   writes received data to an SD file, OR
//   verifies received data against an existing SD file
boolean readHttp2File(char* server, uint16_t port, char* path, char* filename, boolean verify) {

  boolean returnval = false;
  const int kNetworkTimeout = 5*1000;  // msec to wait w/o receiving data before giving up
  const int kNetworkDelay = 1000;      // msec to wait w/o available data before trying again
  
  if ( SD_ok ) {
    if ( !verify ) {
      if ( SD.exists(filename) ) SD.remove(filename);
      Serial << "webclient: readHttp2File DOWNLOADING " << server << ":" << port << path << " to SD:/" << filename << "\n";
    } else {
      Serial << "webclient: readHttp2File VERIFYING " << server << ":" << port << path << " against SD:/" << filename << "\n";
    }
  
    File thefile;
  
    if ( thefile = SD.open(filename, FILE_WRITE) ) {
      Serial << "webclient: readHttp2File Opened SD:/" << filename << " for write\n";

      if ( verify ) thefile.seek(0);
      boolean verify_ok = true;
      int bufindx = 0;
      int rc = 0;
      int num_bytes = 0;
      long time_waiting = 0;
      int file_index = 0;
      auto starttime = millis();
      int num_errors = 0;
      int num_waits = 0;
      
      rc = http.get(server, port, path);
      http.sendHeader("Accept", "application/octet-stream");
      http.endRequest();
      if ( rc == 0 ) {
        rc = http.responseStatusCode();  // 200 = success
         if ( rc == 200 ) {
          rc = http.skipResponseHeaders();
          if ( rc >= 0 ) {
            int bodyLen = http.contentLength();
            Serial << "webclient: readHttp2File contentLength = " << bodyLen << " bytes\n";
 
            unsigned long timeoutStart = millis();
            char c;

            // While we haven't timed out and haven't reached the end of the body
            while ( ( http.connected() || http.available() ) && 
                    ( (millis() - timeoutStart) < kNetworkTimeout ) ) {
              if ( http.available() ) {
                timeoutStart = millis();
                num_waits = 0;
                if ( verify ) {
                  c = http.read();
                  bodyLen--;
                  byte read_value = thefile.read();
                  if ( read_value != c) {
                    verify_ok = false;
                    num_errors++;
                    if ( num_errors < 100 ) {
                      Serial << "webclient: At addr: 0x" << _HEX(file_index) << " expected 0x" << _HEX((int)c) 
                             << " read from file: 0x" << _HEX(read_value) << "\n";
                    }
                  }
                  file_index++;
                } else {  // not verifying... so we read up to 4096 bytes into a buffer
                  int bytes_received = http.readBytes( &rcvbuf[bufindx], BUFSIZE - bufindx );
                  bufindx += bytes_received;
                  bodyLen -= bytes_received;
                  if ( (bufindx >= BUFSIZE) || (bodyLen == 0) ) {
                    //Serial << "webclient: Writing " << bufindx << " bytes to SD.\n";
                    thefile.write(rcvbuf, bufindx);  // write buffer to flash
                    file_index += bufindx;
                    num_bytes += bufindx;
                    if ( bodyLen == 0 ) break;
                    bufindx = 0;
                  }
                }
                timeoutStart = millis();
              } else {  // We don't have any data, so pause to allow some to arrive.
                num_waits++;
                delay(kNetworkDelay);
                time_waiting += kNetworkDelay;
              }
            }
            if ( verify ) {
              if ( verify_ok ) {
                Serial << "webclient: readHttp2File SD file verified.\n";
                returnval = true;
              } else {
                Serial << "webclient: readHttp2File " << num_errors << " ERRORS WERE DETECTED!\n";
              }
            } else {
              returnval = true; 
            }
          } else {
            Serial << "webclient: readHttp2File Failed to skip response headers, rc = " << rc << "\n";
          }
        } else {
          Serial << "webclient: readHttp2File Get response failed, rc = " << rc << "\n";
        }
      } else {
        Serial << "webclient: readHttp2File Connect failed, rc = " << rc << "\n";
      }
      Serial << "webclient: readHttp2File Downloaded " << num_bytes << " bytes in " << (millis() - starttime) << " msec\n";
      Serial << "webclient: readHttp2File Network wait time = " << time_waiting << " msec\n";
      if ( num_bytes == 0 && !verify ) returnval = false;
      http.stop();
      thefile.close();
    } else {
      Serial << "webclient: readHttp2File SD file open failed.\n";
    }
  }  // END if (SD_ok)...
  return( returnval );  
}



// This function writes contents of an SD file to the SAM DUE's on-chip Flash memory.
boolean writeFile2Flash(char* filename, int startaddr) {
  boolean rc = false;
  File thefile;
  int bufaddr = 0;
  int num_bytes = 0;
  int flash_addr = startaddr;
  
  //Serial << "webclient: writeFile2Flash IFLASH0_ADDR = 0x" << _HEX(IFLASH0_ADDR) << "\n";
  //Serial << "webclient: writeFile2Flash IFLASH1_ADDR = 0x" << _HEX(IFLASH1_ADDR) << "\n";
  //Serial << "webclient: writeFile2Flash IFLASH0_SIZE = 0x" << _HEX(IFLASH0_SIZE) << "\n";
  //Serial << "webclient: writeFile2Flash IFLASH1_SIZE = 0x" << _HEX(IFLASH1_SIZE) << "\n";
  //Serial << "webclient: writeFile2Flash FLASH_START = 0x" << _HEX((uint32_t)FLASH_START) << "\n";
  //Serial << "webclient: writeFile2Flash IFLASH0_PAGE_SIZE = " << IFLASH0_PAGE_SIZE << " bytes\n";
  //Serial << "webclient: writeFile2Flash IFLASH1_PAGE_SIZE = " << IFLASH1_PAGE_SIZE << " bytes\n";
  //Serial << "webclient: writeFile2Flash DATA_LENGTH = " << DATA_LENGTH << " bytes\n";
  
  Serial << "webclient: writeFile2Flash Writing to flash at 0x" << _HEX(((uint32_t)FLASH_START+flash_addr)) << "\n";
  if (thefile = SD.open(filename, FILE_READ)) {
    int bytes_remaining = thefile.size();  // compiler allows this, but .size() method used elsewhere caused compiler errors with SdFat
    Serial << "webclient: writeFile2Flash " << filename << " opened for read.\n";
    while ( thefile.available() ) {
      rcvbuf[bufaddr++] = thefile.read();
      bytes_remaining--;
      if (bufaddr >= BUFSIZE || bytes_remaining == 0) {
        //Serial << "webclient: Writing " << bufaddr << " bytes to flash at 0x" << _HEX(((uint32_t)FLASH_START+flash_addr)) << "\n";
        // NOTE: See the change to library file DueFlashStorage.h to allow an ABSOLUTE flash memory address, 
        // e.g., 0x80000 or 0xC0000. The default was an address RELATIVE to IFLASH1_ADDR = 0xC0000.
        dueflashstorage.write( flash_addr, rcvbuf, bufaddr );
        flash_addr += bufaddr;
        bufaddr = 0;
      }
      num_bytes++;
      //if((num_bytes % 100) == 0) Serial.println(num_bytes);
    }
    Serial << "webclient: writeFile2Flash Wrote " << num_bytes << " bytes to flash.\n";
    rc = true;
    thefile.close();
  }
  return(rc);
}



// Read n bytes from flash at startaddr.
void showNFlashBytes(int n, int startaddr) {
  Serial << "webclient: called showNFlashBytes(" << n << ", 0x" << _HEX(startaddr) << "):\n";
  for (int i = 0; i < n; i++) {                     // for n bytes...
    int val = dueflashstorage.read(startaddr + i);  //   read a byte = 8 bits = 0x00-0xFF
    if (val < 0x10) Serial.print(0);                //   if byte val is 0x00-0x0F, print a leading 0
    Serial.print(val, 16);                          //   print hex value of the byte
    if ( (i+1)%4 == 0 ) {                           //   if we've printed 4 bytes...
      Serial.print(" | ");                          //     print a divider
    } else {                                        //   otherwise...
      Serial.print(" ");                            //     print a space
    }
  }
  Serial.println("");                               // print a newline char
}



// Get the GPNVM bits
uint32_t getGPNVMBits(Efc* p_efc) {
  // When reading GPNVM, the bit number is irrelevant.
  if ( EFC_RC_OK != efc_perform_command(p_efc, EFC_FCMD_GGPB, 0) ) {
    return FLASH_RC_ERROR;
  }
  return( efc_get_result(p_efc) );
}



// Get Ethernet status. 
//   ***Used in wwe.ino to set the ethernet_ok (global) flag***
//   Ethernet.linkStatus() is a HARDWARE test. It will fail if the Ethernet cable is unplugged or bad.
//   client.connect() is a SOFTWARE test to check if we can connect to a server.
boolean EthernetOK() {
  char cfg_addr[20];            // Update Server IP address string, e.g., "192.168.1.4"
  uint16_t cfg_port;            // Update Server port, e.g., 80 or 49152
  boolean ethernetOK = false;   // status flag
  
  strcpy(cfg_addr, parm_cfg_ip.parmVal());  // get Update Server IP address from its parm
  cfg_port = parm_cfg_port.intVal();        // get Update Server port from its parm

  auto connecttime = micros();
  if ( client.connect(cfg_addr, cfg_port) ) {
    Serial << "webclient: Ethernet.linkStatus() = " << Ethernet.linkStatus() << "\n";
    Serial.print("webclient: testEthernet client.connect time = ");
    Serial.print( ((float)(micros() - connecttime)/1000.), 3 );
    Serial.println(" msec");
    ethernetOK = true;
  } else {
    Serial << "webclient: Ethernet.linkStatus() = " << Ethernet.linkStatus() << "\n";
    Serial << "webclient: testEthernet FAIL!\n";
    ethernetOK = false;
  }
  client.stop();  // returns nothing
  return( ethernetOK );
}
