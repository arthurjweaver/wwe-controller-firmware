// ---------- webserver.ino ----------
//
// The HTTP server responds to the following commands:
//   http://<controllerIP>/wave.json --> returns a JSON string that contains waveform data for the analog
//      channels defined in graph_channels[] - see web.ino and webserver.ino
//   http://<controllerIP>/measure.json --> returns a JSON string containing all channel RMS values.
//
//   HTTP POST to setvals.json, used in test mode, sets the values of all analog inputs
//     to the values contained in the JSON array contained in the body of the POST.

// These webserver functions are called when the server receives a corresponding command - defined by .addCommand() below
void waveCmd(WebServer&, WebServer::ConnectionType, char*, bool);
void measureCmd(WebServer&, WebServer::ConnectionType, char*, bool);
void failCmd(WebServer&, WebServer::ConnectionType, char*, bool);
void setvalsCmd(WebServer&, WebServer::ConnectionType, char*, bool);
void windStreamCmd(WebServer&, WebServer::ConnectionType, char*, bool);
void windCfgCmd(WebServer&, WebServer::ConnectionType, char*, bool);
void parmCmd(WebServer&, WebServer::ConnectionType, char*, bool);
void statusCmd(WebServer&, WebServer::ConnectionType, char*, bool);
void modbus1Cmd(WebServer&, WebServer::ConnectionType, char*, bool);


void initServer(){
  Serial << "webserver: Starting webserver...\n";
  webserver.begin();
  
  // Register commands for web server
  webserver.setDefaultCommand(&failCmd);               // Default response from webserver if no specific command is given: server:port/<command>
  webserver.setFailureCommand(&failCmd);
  webserver.addCommand("parms.html", &parmCmd);        // Show a web form with controller operating parms
  // Disable everything else:
  //webserver.addCommand("wave.json", &waveCmd);         // Return waveforms
  //webserver.addCommand("measure.json", &measureCmd);   // Return collected RMS values
  //webserver.addCommand("setvals.json", &setvalsCmd);   // Set values for testing
  //webserver.addCommand("windstream", &windStreamCmd);  // Send "stream" command to Etesian
  //webserver.addCommand("windsetcfg", &windCfgCmd);     // Retrieve Etesian configuration
  //webserver.addCommand("status.html", &statusCmd);     // Show a web page with analog channel vals, updated every 5 sec
  //webserver.addCommand("modbus1.html", &modbus1Cmd);   // Show a web page with all modbus 'fast' channel vals, updated every 5 sec
}



//================================//
//===== Web server functions =====//
//================================//

// This function prints waveform data in JSON format
// For example: {"label":"V12","yaxis":1,"data":[[0,2],[1,3],[2,2],[3,1],[4,0]]}
void printChannelWaveJSON(WebServer &server, int channel, int axis_num) {

  float time_offset = 0.0;
  float* data = getChannelWaveformData(channel);         // see adc.ino
  server.printP("{\"label\":\"");                        // prints: {"label":" 
  server.printP(getChannelName(channel));                // prints channel label
  server.printf("\",\"yaxis\":%d", axis_num);            // prints: ","yaxis":#
  server.printP(",\"data\":[");                          // prints: ,"data":[
  
  for(int i = 0; i < SAMPLES_IN_WAVEFORM; i++) {         // for each waveform to be sampled... 
    if (i > 0) server.printP(",");                       //   if this isn't the first data point, print a comma between data points
    server.printf("[%.1f,%.2f]", time_offset, data[i]);  //   print a single data point formatted as: [x.x,y.yy]
    time_offset += MAIN_SAMPLE_PERIOD_MILLIS;            //   increment the time (+1 msec)
  }
  server.printP("]}");                                   // print closing ] of "data":[ ] and final }
}


// Respond to a GET with a JSON string that contains current waveform data.
void waveCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete){
  //suspend_post = true;
  // example:
  // [{label: "L1L2 Voltage", data: [[0,1],[1,2],[2,3],[3,2],[4,1]]},{label: "L2L3 Voltage", data: [[0,2],[1,3],[2,2],[3,1],[4,0]]}] 
  
  //dbgPrintln(1, "web: doing waveCmd");
  //unsigned long starttime = micros();
  startCollectingWaveforms();
  while (checkCollectingWaveforms() == true) delay(10);
  //unsigned long elapsed = micros() - starttime;
  //dbgPrintln(1, elapsed);
  
  //dbgPrintln(1, "web: ok, do the access");  
  // So far, this only works with flot, Jquery.get() and text/html.
  // Seems like it should be JQuery.getJSON and application/json.
  server.httpSuccess("Content-Type: text/html");
  server.printP("[");
  //Serial.println("go get the data");
  for (int j = 0; j < NUM_GRAPH_CHANNELS; j++) {
    int i = graph_channels[j];
    //dbgPrint(1, "web: process channel ");
    //dbgPrintln(1, i);
    if (j > 0) server.printP(", ");
    printChannelWaveJSON(server, i, axis_nums[j]);
  }
  //dbgPrintln(1, "web: OK, finished the accesses");

  server.printP("]");
  //suspend_post = false;
}


// Return a JSON string with current analog chanel vals: [{"<channel_name>": <rms_val>, etc.}]
void measureCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete){
  dbgPrintln(1, "web: doing measureCmd");
  server.httpSuccess("Content-Type: application/json");
  server.printP("[{");
  for (int i = 0; i < 14; i++) {
    if(i > 0) server.printP(", ");
    server.printf("\"%s\": %.2f", getChannelName(i), (float)getChannelRMSInt(i) / 1024.0);
  }
  server.printP("}]");
}


// Get Etesian anemometer config, return it, then restart stream mode
void windCfgCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete){
  dbgPrintln(1, "web: doing windConfigCmd");
  server.httpSuccess("Content-Type: text/plain");
  Serial2.write(0x1b);  // send escape to knock out of stream mode
  Serial2.println("config");
  while(Serial2.available()){
    int inByte = Serial2.read();
    if(inByte == '$') break;
    server.write((char)inByte);
  }
  Serial2.println("stream");
}


// Put Etesian anemometer into stream mode
void windStreamCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete){
  dbgPrintln(1, "web: doing windStreamCmd");
  server.httpSuccess("Content-Type: text/plain");
  Serial2.println("stream");
  server.printP("OK");
}


// This function takes data presented in the body of a POST and puts it into an array in adc.ino.
// This is for TESTING ONLY, in place of what is read from the ADC. 
// The body of the POST should look like this:
//   [1.2,2.3,3.4,4.5,5.6,6.7,7.8,8.9,9.1,10.2,11.3,12.4]
// If the module is compiled with the USE_TEST_VALS defined, these values will be put into the ADC module 
//   and used when getChannelRMSInt(int channel) is called, in place of measured values.
// Channels:
// 0 l1 voltage
// 1 L2 Voltage
// 2 L3 Voltage
// 3 DC Voltage
// 4 Line Voltage
// 5 L1 Current
// 6 L2 Current
// 7 L3 Current
// 8 DC Current
// 9 L1L2 Voltage
// 10 L2L3 Voltage
// 11 L3L1 Voltage
// 12 Frequency
// 13 Wind Speed (m/s)
void setvalsCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (type == WebServer::POST){
    int c;
    char buf[10];

    int charnum = 0;
    int bufptr = 0;
    int num_num_chars = 0;
    int num_delimiters = 0;
    float vars[14];
    int varptr = 0;
    
    while((c = server.read()) != -1) {
      if((c >= 0x30 && c <= 0x39) || c == 0x2e){
        buf[bufptr++] = (char)c;
        //Serial.write(c);
        num_num_chars++;  
      } else {
        // Non-numeric char takes us here.
        // We null-terminate the string, then convert the string to a float.
        if (bufptr > 0) {
          buf[bufptr] = 0;
          bufptr = 0;
          // 
          float val = atof(buf);
          if(varptr < 14){
            setTestValue(varptr, val);
            vars[varptr++] = val;
          }
        }
        num_delimiters++;
      }
    }
    server.printP("OK");
    for(int i = 0; i < varptr; i++){
      dbgPrintln(1, vars[i]);
    }
  }else{
    server.printP("ERROR");
  }
}


// This function creates an HTML *form* containing Controller Operating Parameters and handles user-submitted updates to parm vals.
// Clicking the form's "Submit" button initiates a POST that sends updated parms back to this code which saves parm vals and writes them to SD.
void parmCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  char parmval[40];
  int write_col = 0;

  // Handle user-updated parms from the Controller Operating Parameters web form.
  if ( type == WebServer::POST ) {
    boolean repeat;         // data available flag
    Parm* theparm;          // pointer to a parm object
    char key[30], val[32];  // char arrays for parm key (parm name) and val (parm val) pairs
    
    Serial << "webserver: parmCmd POST Controller Operating Parameters from submitted form...\n";
    do {
      repeat = server.readPOSTparam(key, 30, val, 32);    // read a parm into key[30] and val[32] char arrays
      if ( repeat ) {                                     // if we've read a parm...
        if ( theparm = findParm(key) ) {                  //   get pointer to a parm object - see parms.h
          if ( (!strcmp(key, "furl_init_windspeed")) || 
               (!strcmp(key, "sc_emer_windspeed")) ) {    //     for those parms with MPH units on the parms page...
            float msval = (float)(atof(val)*MPH2MS);      //       convert mph --> m/s
            theparm->setParmVal(msval);                   //       save parm val in m/s units
            Serial << key << " = " << msval << " m/s\n";  //       print parm name and val (m/s)
          } else {                                        //     otherwise, 
            theparm->setParmVal(val);                     //       save parm val
            Serial << key << " = " << val << "\n";        //       print parm name and val
          }
        }
      }
    } while (repeat);
    
    server.httpSeeOther(PREFIX "/parms.html");  // display the updated page
    writeParms2SD(PARMFILENAME);                // save updated parms to SD
    
  } else {  // this is not a POST, so... it's a GET
    
    // DISPLAY the parm page on a browser. This happens when we RELOAD the page.
    if ( type == WebServer::GET ) Serial.println("webserver: parmCmd GET Controller Operating Parameters web form...");

    // generate an HTML header
    P(htmlHead) =
    "<html>"
    "<head>"
    "<title>Controller Operating Parameters</title>"
    "<style type=\"text/css\">"
    "BODY { font-family: sans-serif }"
    "H1 { font-size: 14pt; }"
    "P  { font-size: 10pt; }"
    "</style>"
    "</head>"
    "<body>";
    
    server.httpSuccess();     // 
    server.printP(htmlHead);  // print HTML header to server
    server << "<form action='" PREFIX "/parms.html' method='post'>";
    server << "<h1>Controller Operating Parameters</h1>\n";
    server << "<table>";
    
    // Create a 7-column table: parmname | parmval | parmunits | empty col | parmname | parmval | parmunits
    for ( int i = 0; i < num_parms; i++ ) {
      Parm* theparmptr = parmary[i];                                  // get i'th parm from the parm array - see parms.h
      char* parmname = theparmptr->parmName();                        // get i'th parm name
      strcpy(parmval , theparmptr->parmVal());                        // get i'th parm val as string
      if ( (!strcmp(parmname, "furl_init_windspeed")) || 
           (!strcmp(parmname, "sc_emer_windspeed")) ) {               // for those parms with MPH units on the parms page...
        int newval = round(theparmptr->floatVal()*MS2MPH);            //   convert WS parm float val to m/s and round to nearest int
        sprintf(parmval, "%d", newval);                               //   display WS parm int val
      }

      // partition table into parameter groups - see parmdefs.h
      if ( i==0 ) server << "<tr><td colspan=7 style='text-align:center; background-color:#f0f0f0; font-style:italic'>Turbine control</td></tr><tr>";  // 14 parms, 0-13
      if ( i==14 ) server << "<tr><td colspan=7 style='text-align:center; background-color:#f0f0f0; font-style:italic'>Network</td></tr><tr>";         // 10 parms, 14-23
      if ( i==24 ) server << "<tr><td colspan=7 style='text-align:center; background-color:#f0f0f0; font-style:italic'>Diversion load</td></tr><tr>";  // 5 parms, 24-28
      if ( i==29 ) {
        write_col = 0;
        server << "</tr>"; // add </tr> because i is odd
        server << "<tr><td colspan=7 style='text-align:center; background-color:#f0f0f0; font-style:italic'>Local site</td></tr><tr>";  // 7 parms, 29-35
      } 
      
      server << "<td>" << theparmptr->parmEngName() << "</td>";                  // print parm display name
      if ( !strcmp(parmname, "shutdown_state") ) {                               // this parm gets a drop-down selection with *3* options
        server << "<td><select name='" << theparmptr->parmName() << "'>"; 
        if ( !strcmp(parmval, "0") ) server << "<option value='0' selected>Normal Operation</option><option value='1'>Shutdown - routine</option><option value='2'>Shutdown - EMERGENCY</option>";
        if ( !strcmp(parmval, "1") ) server << "<option value='0'>Normal Operation</option><option value='1' selected>Shutdown - routine</option><option value='2'>Shutdown - EMERGENCY</option>";
        if ( !strcmp(parmval, "2") ) server << "<option value='0'>Normal Operation</option><option value='1'>Shutdown - routine</option><option value='2' selected>Shutdown - EMERGENCY</option>";
        server << "</td><td></td>";                                              // no parm units needed for <select> drop-downs, so insert <td></td>
      } else if ( !strcmp(parmname, "ovrd") ||                                   // these parms get a drop-down selection with *2* options
                  !strcmp(parmname, "hvdl_active") ||
                  !strcmp(parmname, "pv1_disc") ||
                  !strcmp(parmname, "pv2_disc") ||
                  !strcmp(parmname, "wx_override") ) {
        server << "<td><select name='" << theparmptr->parmName() << "'>"; 
        if ( !strcmp(parmval, "0") ) server << "<option value='0' selected>No</option><option value='1'>Yes</option>";
        if ( !strcmp(parmval, "1") ) server << "<option value='0'>No</option><option value='1' selected>Yes</option>";
        server << "</td><td></td>";
     } else {                                                                   // all other parms get a text field then parm units
        server << "<td><input type='text' name='" << theparmptr->parmName()
               << "' value='" << parmval << "'></td><td>" << theparmptr->parmUnits() << "</td>";
      }

      write_col==0 ? server << "<td>&nbsp&nbsp</td>" : server << "</tr>\n";  // if we're writing to column 0, add empty <td>, otherwise end the row
      write_col==1 ? write_col=0 : write_col=1;  // toggle 
    }

    server << "<tr><td colspan=7>&nbsp;</td></tr>";  // insert blank row
    server << "<tr><td colspan=7><input type='submit' value='Submit' style='color:white; background-color:red; width:100%'/></td></tr>";  // put Submit button in last table row
    server << "</table>";  // close <table>
    server << "</form>";   // close <form>
    server << "</body>";   // close <body>
  }
}



void showFurlReasons(WebServer &server) {
  int furl_reason = getFurlReason();
  server << " ";
  if(furl_reason & 1) server << "V";
  if(furl_reason & 2) server << "I";
  if(furl_reason & 4) server << "F";
  if(furl_reason & 8) server << "W";
}


void statusCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  Serial.println("entering statusCmd");
  if (type == WebServer::GET){
    Serial.println("doing GET");
    P(htmlHead) =
    "<html>"
    "<head>"
    "<title>WWE Controller Status</title>"
    "<style type=\"text/css\">"
    "BODY { font-family: sans-serif }"
    "H1 { font-size: 14pt; text-decoration: underline }"
    "P  { font-size: 10pt; }"
    "</style>"
    "<meta http-equiv='refresh' content='5' >"
    "</head>\n"
    "<body>\n";
    
    server.httpSuccess();
    server.printP(htmlHead);
    
    server << "\n<p>" << url_tail << "\n<p>Furl State: \n";
    int furl_state = getFurlState();
    if(furl_state == UNFURL) {
      server << "UNFURL, ";
    }else{
      server << "FURL, ";
    }
    server << furl_motor_on;
    server << "<br>";
    for(int i = 3; i < 14; i++){
      server << getChannelName(i) << " = " << getChannelRMSInt(i)  << "<br>\n";
    }
    server << "</body>";
  }
}


void failCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete ) {
  server.httpFail();  // sends "HTTP 400 - Bad Request" headers back to the browser
}


void modbus1Cmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  Serial.println("entering modbusCmd");
  if (type == WebServer::GET){
    Serial.println("doing GET");
    P(htmlHead) =
    "<html>"
    "<head>"
    "<title>WWE Modbus Read</title>"
    "<style type=\"text/css\">"
    "BODY { font-family: sans-serif }"
    "H1 { font-size: 14pt; text-decoration: underline }"
    "P  { font-size: 10pt; }"
    "</style>"
    "<meta http-equiv='refresh' content='5' >"
    "</head>\n"
    "<body>\n";
    
    server.httpSuccess();
    server.printP(htmlHead);
    server << "<H1>Modbus real-time data<\H1>\n";
    server << "<p><ul>\n";
    for (int i = 0; i< NUM_MOD_FAST_CHANNELS; i++) {
      server << "<li>" << mod_fast_regs[i]->getChanName() << " = " << mod_fast_regs[i]->valStrg() << " " << mod_fast_regs[i]->getUnits() << "\n";
    }
    server << "</ul></p></body></html>\n";
  }
}
