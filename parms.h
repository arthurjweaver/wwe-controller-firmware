// ---------- parms.h ----------
//
#ifndef PARMS_DEFINED

#define TYPE_STR 0
#define TYPE_INT 1
#define TYPE_FLOAT 2
#define TYPE_IP 3
#define MAX_PARMS 40

class Parm;
int num_parms = 0;
boolean addParm(Parm*);
boolean parms_dirty = false;
void digitalWriteDirect(int, boolean);

void setParmsDirty() {
  //digitalWriteDirect(39, HIGH);
  parms_dirty = true;
  //digitalWriteDirect(39, LOW);
}

class Parm {
  public:
    // Constructors.
    //
    // STRING parameter:
    // SPACE FOR THE STRING MUST BE PREALLOCATED AND GIVEN TO THIS CONSTRUCTOR!
    // Although Parm has its own string buffer, forcing use of an external one allows use of strings longer than the fixed length of the internal buffer.
    Parm(char* parmname, char* engname, char* val):
         parmname(parmname), strval(val), parm_eng_name(engname), parm_units("") {  // arg initializers
      parmtype = TYPE_STR;  // set parm type
      addParm(this);        // add to the parameter list
      newparm = true;       // set flag
      setParmsDirty();      //
    }

    // INTEGER parameter:
    Parm(char* parmname, char* engname, char* units, int val): 
         parmname(parmname), intval(val), parm_eng_name(engname), parm_units(units) {  // arg initializers
      parmtype = TYPE_INT;
      addParm(this);
      sprintf(strbuf, "%d", val);  // string representation; a terminating null char is automatically added
      newparm = true;
      setParmsDirty();
    }
    
    // INTEGER parameter with max/min limits:
    Parm(char* parmname, char* engname, char* units, int val, int maxval, int minval): 
         parmname(parmname), intval(val), parm_eng_name(engname), parm_units(units), int_range_max(maxval), int_range_min(minval) {  // arg initializers
      parmtype = TYPE_INT;
      addParm(this);
      sprintf(strbuf, "%d", val);  // string representation
      newparm = true;
      setParmsDirty();
    }
    
    // FLOAT parameter:
    Parm(char* parmname, char* engname, char* units, float val): 
         parmname(parmname), floatval(val), parm_eng_name(engname), parm_units(units) {  // arg initializers
      parmtype = TYPE_FLOAT;
      addParm(this);
      sprintf(strbuf, "%0.1f", val);            // string representation
      floatval_int = (int)(1024.0 * floatval);  // int representation; used in checkSCNowConditions() - see furlctl.ino
      newparm = true;
      setParmsDirty();
    }

    // IP address parameter, specified as an IPAddress:
    Parm(char* parmname, char* engname, IPAddress ipaddr): 
         parmname(parmname), ipval(ipaddr), parm_eng_name(engname), parm_units(units) {  // arg initializers
      parmtype = TYPE_IP;
      addParm(this);
      sprintf(strbuf, "%d.%d.%d.%d", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);  // string representation
      newparm = true;
      setParmsDirty();
    }

    void setEngName(char* engname) {
      strcpy(parm_eng_name, engname);
    }
    
    void setUnits(char* engname) {
      strcpy(parm_eng_name, engname);
    }

    // Set value of STRING parm; return true if successful, false if not.
    // If parm type is string, just copy it to the Parm's buffer, otherwise, convert it.
    boolean setParmVal(char* val) {
      if (parmtype == TYPE_STR) {
       //strcpy(strbuf, val);                       // string representation; why commented out?
       strcpy(strval, val);                       // ???
      }
      if (parmtype == TYPE_INT) {
        strcpy(strbuf, val);                      // string representation
        intval = atoi(val);                       // int representation
      }
      if (parmtype == TYPE_FLOAT) {
        strcpy(strbuf, val);                      // string representation
        floatval = atof(val);                     // float representation
        floatval_int = (int)(1024.0 * floatval);  // int representation
      }
      
      if (parmtype == TYPE_IP) {
        strcpy(strbuf, val);                                                        // string representation
        sscanf(strbuf, "%d.%d.%d.%d", &ipval[0], &ipval[1], &ipval[2], &ipval[3]);  // array representation
      }
      
      newparm = true;
      setParmsDirty();
    }

    // Set value of INTEGER parm; return true if successful, false if not.
    boolean setParmVal(int val) {
      //Serial << "parms.h: set INTEGER parm to " << val << "\n";
      sprintf(strbuf, "%d", val);
      intval = val;
      newparm = true;
      setParmsDirty();
    }

    // Set value of FLOAT parm; return true if successful, false if not.
    boolean setParmVal(float val) {
      //Serial << "parms.h: set FLOAT parm to " << val << "\n";
      sprintf(strbuf, "%.2f", val);
      floatval = val;
      floatval_int = (int)(1024.0 * floatval);
      newparm = true;
      setParmsDirty();
    }

    // Set value of IPAddress parm; return true if successful, false if not.
    boolean setParmVal(IPAddress ip) {
      sprintf(strbuf, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
      ipval = ip;
    }
    
    // Return string value of TYPE_STR parm
    char* parmVal() {
      if (parmtype == TYPE_STR) {
        return(strval);
      } else {
        return(strbuf);
      }
    }
    
    // Return int value of TYPE_INT parm
    int intVal() {
      return(intval);
    }

    // Return float value of TYPE_FLOAT parm
    float floatVal() {
      return(floatval);
    }

    // Return int value of TYPE_FLOAT parm; as (int)(1024.0 * floatval)
    int floatValInt() {
      return(floatval_int);
    }

    // Return IPAddress of TYPE_IP parm, e.g., parm_controller_ip.IPVal() returns an IPAddress object
    IPAddress IPVal() {
      return(ipval);
    }

    // Return string value of parmName
    char* parmName(){
      return(parmname);
    }

    // Return string value of parmEngName
    char* parmEngName() {
      return(parm_eng_name);
    }

    // Return string value of parmUnits
    char* parmUnits() {
      return(parm_units);
    }

    // Check the parm has a new value, clear the flag.
    boolean checkNewParm() {
      boolean getnewparm = newparm;
      newparm = false;
      return(getnewparm);
    }

    // Return int indicating parm type
    int parmType() {
      return(parmtype);
    }
    
  private:
    int int_range_max;
    int int_range_min;
    float float_range_max;
    float float_range_min;
    boolean newparm;
    char strbuf[40];
    int parmtype;
    char* parmname;
    char* strval;
    char* parm_eng_name;
    char* parm_units;
    int intval;
    float floatval;
    int floatval_int;
    char* units;
    IPAddress ipval;
};  // end class Parm{};


// A pointer to each parm is added to this array as they are created. This gives us a way to read and write all of them.
Parm* parmary[MAX_PARMS];

int findParmIndex(char* pn){
  //Serial.print("findParmIndex(): ");
  //Serial.println(pn);
  for(int i = 0; i < num_parms; i++){
    Parm* parmptr = parmary[i];
    char* parmname = parmptr->parmName();
    //Serial.println(parmname);
    if(strcmp(parmname, pn) == 0){
      //Serial.println("found");
      return(i);
    }
  }    
  //Serial.println("not found");
  return(-1);
}

Parm* findParm(char* pn){
  for(int i = 0; i < num_parms; i++){
    Parm* parmptr = parmary[i];
    char* parmname = parmptr->parmName();
    //Serial.println(parmname);
    if(strcmp(parmname, pn) == 0){
      return(parmptr);
    }
  }    
  //Serial.println("not found");
  return(NULL);
}

// This version always returns the string version of the parm.
    
char* parmVal(char* pn){
  int i = findParmIndex(pn);
  if(i >= 0){
    Parm* parmptr = parmary[i];
    return(parmptr->parmVal());
  }else{
    return("not found");
  }
}

boolean setParmVal(char* pn, char* val){
  int i = findParmIndex(pn);
  //Serial.print("setParmVal():  ");
  //Serial.print(pn);
  //Serial.print(" = ");
  //Serial.println(val);
  if(i >= 0){
    //Serial.println(i);
    Parm* parmptr = parmary[i];
    //Serial.println(parmptr->parmName());
    parmptr->setParmVal(val);
    return(true);
  }else{
    return(false);
  }
}

boolean addParm(Parm* newparmptr){
  if(num_parms < MAX_PARMS){
    parmary[num_parms++] = newparmptr;
    return(true);
  }else{
    return(false);
  }
}
#endif
#define PARMS_DEFINED
