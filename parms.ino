// ---------- parms.ino ----------
// Parms are operating parameters that need to be saved in nonvolatile memory.
// This module contains functions to move them around and print them:
// 
// boolean writeParms2SD(char* filename)      : current parms --> buffer --> SD, used in wwe.ino and by parmCmd() in webserver.ino
// void writeParms2Buff(boolean include_mac)  : current parms --> buffer, used by writeParms2SD(), and in wwe.ino

// boolean readSD2Parms(char* filename)       : SD --> buffer --> current parms, used in wwe.ino
// int writeBuff2Parms()                      : buffer --> current parms, used by readSD2Parms(), and by sendConfigUDP() in web.ino

// void printParms()                          : current parms --> Serial
// void printJSONBuf()                        : buffer --> Serial


// WHAT'S THIS?
using namespace ArduinoJson::Parser;
JsonParser<100> parser;


// Write current parms --> jsonbuf --> SD, overwriting an existing parm file (if any).
boolean writeParms2SD(char* filename) {
  File parmfile;

  if ( SD_ok ) {                                       // if SD is OK...
    if ( SD.exists(filename) ) {                       //   if parm file EXISTS...
      if ( SD.remove(filename) ) {                     //     if we successfully remove it...
        Serial << "parms: writeParms2SD(): Removed EXISTING file: " << filename << "\n";
      } else {                                         //     otherwise...
        Serial << "parms: writeParms2SD(): Cannot remove: " << filename << "\n";
        return(false);                                 //       end in failure.
      }
    }
    // Either there's no existing parm file or we've just removed it, so...
    if ( parmfile = SD.open(filename, FILE_WRITE) ) {  //   if a new parm file can be opened...
      Serial << "parms: writeParms2SD(): Opened NEW file: " << filename << "\n";
      writeParms2Buff(false);                          //     current parms --> jsonbuf (false = don't include MAC)
      parmfile.println(jsonbuf);                       //     jsonbuf --> SD
      parmfile.close();                                //     close file
      //printJSONBuf();                                  //     print jsonbuf
      parms_dirty = false;                             //     reset flag. This is the ONLY place where parms_dirty is used. Not useful anymore?
      Serial << "parms: writeParms2SD(): Wrote current parms to: " << filename << "\n";
      return(true);                                    //     we've succeeded!
    } else {                                           //   otherwise, a new parm file can't be opened...
      Serial << "parms: writeParms2SD(): Error opening: " << filename << "\n";
      return(false);                                   //     end in failure.
    }
  } else {                                             // otherwise, SD is not ok...
    Serial << "parms: writeParms2SD(): SD is NOT OK!\n";
    return(false);                                     //   end in failure.
  }
}


// Write current parms --> jsonbuf
// This will be of the form: {"info":{"k1":"v1", <etc>}, "config":{"key1":"val1", "key2":"val2" <,etc>}}
// This function is an excellent example of C++ string manipulation!
// if jsonbuf is being used for udp config, use writeParms2Buff(true) to include controller MAC 
void writeParms2Buff(boolean include_mac) {
  char *bufptr = jsonbuf;  // bufptr is a pointer to char, initialized to the address of another char variable, which is jsonbuf.
                           // *bufptr is the contents of what is pointed to, i.e., the jsonbuf string (char array).
                           // jsonbuf has already been initialized as a GLOBAL var, char jsonbuf[4096] - see wwe.ino.
                           // In the code below, as we write chars to *bufptr (==jsonbuf) with strcpy() or sprintf(), 
                           // we have to increment bufptr by the length of the added string. 
                           // Or, to add just a single char, then *bufptr++ = "{" will do it.
  char tmpbuf[100];
  
  *bufptr++ = '{';                       // write { to jsonbuf, then increment pointer
  if (include_mac) {                     // if jsonbuf is being used for udp config, include controller MAC 
    int len0 = sprintf(tmpbuf, "\"%s\":\"%s\",", "id", mac_chars);  // create a string like: "id":"80":"1f":"12":"32":"6c":"b6"
    strcpy(bufptr, tmpbuf);              // copy MAC string to jsonbuf
    bufptr += len0;                      // move pointer to string end
  }
  strcpy(bufptr, "\"config\":{");        // write "config":{ to jsonbuf
  bufptr += 10;                          // move pointer to string end
  for (int i = 0; i < num_parms; i++) {  // loop through all parms in parmary[]...
    Parm* theparmptr = parmary[i];
    int len = sprintf(tmpbuf, "\"%s\":\"%s\"", theparmptr->parmName(), theparmptr->parmVal());  // create a string like: "parmname1":"parmval1"
    strcpy(bufptr, tmpbuf);              //   copy parm string to jsonbuf
    bufptr += len;                       //   move pointer to string end
    if (i < num_parms - 1) {             //   if we have more parms...
      *bufptr++ = ',';                   //     write , then increment pointer
    }
  }
  *bufptr++ = '}';                       // write  } , then increment pointer
  *bufptr++ = '}';                       // write  } , then increment pointer
  *bufptr = 0;                           // finish by null terminating the string
}


// Read SD --> jsonbuf --> current parms
boolean readSD2Parms(char* fname) {
  File parmfile;
  int jsonidx = 0;
  
  if ( SD_ok ) {                                                  // if SD is OK...
    if ( SD.exists(fname) ) {                                     //   if parm file EXISTS...
      if ( parmfile = SD.open(fname) ) {                          //     if we successfully open the file...
        Serial << "parms: readSD2Parms(): Opened " << fname << "\n";
        while ( parmfile.available() ) {                          //       SD --> jsonbuf
          jsonbuf[jsonidx++] = parmfile.read();                   //         read one byte (char) at a time into jsonbuf, then increment jsonidx
        }                                                         //
        jsonbuf[jsonidx] = 0;                                     //       null terminate jsonbuf
        Serial << "parms: readSD2Parms(): Finished reading " << fname << " into buffer\n";
        parmfile.close();                                         //       close the file
        writeBuff2Parms();                                        //       jsonbuf --> current parms
        return(true);                                             //       we've succeeded!
      } else {                                                    //     otherwise, the file didn't open...
        Serial << "parms: readSD2Parms(): Error opening " << fname << "\n";
        return(false);                                            //       end in failure.
      }
    } else {                                                      //   otherwise, parm file does NOT exist...
      Serial << "parms: readSD2Parms(): Can't find " << fname << "\n";
      return(false);                                              //     end in failure.
    }
  } else {                                                        // otherwise, SD is not ok...
    Serial << "parms: readSD2Parms(): SD is NOT OK!\n";
    return(false);                                                //   end in failure.
  }
}


// Write jsonbuf --> current parms
int writeBuff2Parms() {
  char tmpkey[40];
  int rc = 0;
  
  JsonObject root = parser.parse(jsonbuf);
  if ( !root.success() ) {
    Serial << "parms: writeBuff2Parms(): buffer parsing FAILED!\n";
  } else {
    //Serial.print("parms: writeBuff2Parms() Buffer parsed OK. Root key = ");
    //for (JsonObjectIterator j=root.begin(); j!=root.end(); ++j) Serial.println(j.key());  // print the root key

    JsonObject config = root["config"];
    for (JsonObjectIterator i=config.begin(); i!=config.end(); ++i) {  // loop through the JSON object...
      //Serial << "parms: writeBuff2Parms() " << i.key() << " = " << (char*)i.value() << "\n";
      strcpy(tmpkey, i.key());                                         //   copy key into tmpkey
      if ( strcmp(tmpkey, "status") != 0 ) {                           //   if tmpkey != "status"...
        setParmVal(tmpkey, i.value());                                 //     ***update parm val to that found in jsonbuf***
        rc |= 1;                                                       //     set return code bit 0 --> parm update has occurred
        Serial << "parms: writeBuff2Parms() Updated parm " << tmpkey << " = " << (char*)i.value() << "\n";
        if (strcmp(tmpkey, "binary_filename") == 0) {                  //     if key == "binary_filename"
          rc |= 2;                                                     //       set return code bit 1 --> firmware update is available
          Serial << "parms: writeBuff2Parms() Firmware update " << (char*)i.value() << " is available!\n";
        }
      }
    }
  if ( rc == 0 ) Serial << "parms: writeBuff2Parms(): Parms unchanged.\n";
  }
  return(rc);
}


// Print current parms to the serial monitor
void printParms() {
  for (int i = 0; i < num_parms; i++) {
    Parm* theparmptr = parmary[i];
    Serial << "parms: parmname=" << theparmptr->parmName() 
           << ", parmType=" << theparmptr->parmType() 
           << ", parmVal=" << theparmptr->parmVal() 
           << ", floatVal=" << theparmptr->floatVal() 
           << ", floatValInt=" << theparmptr->floatValInt() 
           << ", intVal=" << theparmptr->intVal() << "\n";
  }
}


// Print jsonbuf to Serial
void printJSONBuf() {
  Serial.println("parms: jsonbuf =");
  Serial.println(jsonbuf);
}
