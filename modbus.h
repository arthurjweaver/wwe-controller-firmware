// ---------- modbus.h ----------
// Define all of the Modbus channels and groups of channels used for various purposes.

#ifndef modbus_h
#define modbus_h

//-------------------------------------------------------------------------
//  Class definitions
//-------------------------------------------------------------------------
// Data types
#define MOD_HALFWORD 0
#define MOD_FULLWORD 1
#define MOD_FLOAT16 2
#define MOD_FLOAT32 3
#define MOD_STRING 4
// Use scale factors written into the ModbusMasterScaled object
#define MOD_SCALED_I 5
#define MOD_SCALED_V 6
#define MOD_SCALED_P 7
// Uses scale factor stored in the channel object
#define MOD_SCALED 8
#define MOD_LONG_READ 9
#define MOD_HALFWORD_SIGNED 10
#define MOD_HALFWORD_SIGNED_SCALED 11
// Types of Modbus Register
#define MOD_INPUT_REG 0
#define MOD_HOLDING_REG 1
#define MOD_NOTHING 2
// Types of Modbus 
#define MODBUS_TYPE_RTU 0
#define MODBUS_TYPE_TCP 1

class ModbusMasterTCP: public ModbusTCP {
  private:
    Parm& ip_parm;
    boolean cached_data_ok; 
    float scalei;
    float scalev;
    char* name;

  public:
    ModbusMasterTCP(char* name, Parm& ip_parm):ModbusTCP(), name(name), ip_parm(ip_parm), scalei(1.0), scalev(1.0) {
      setServerIPAddress(ip_parm.IPVal());
      cached_data_ok = false;
    }

    // Set the IP Address to the current value of the parameter.
    void updateIPAddress() {
      setServerIPAddress(ip_parm.IPVal());  // setServerIPAddress() is a method in ModbusTCP.h
      //Serial << "modbus.h: setServerIPAddress = " << ip_parm.IPVal() << "\n";
    }
    void setScaleI(float s) {
      scalei = s;
    }
    void setScaleV(float s) {
      scalev = s;
    }
    float getScaleI() {
      return(scalei);  
    }
    float getScaleV() {
      return(scalev);
    }
    boolean cachedDataOK() {
      return(cached_data_ok);  // getter
    }
    void cachedDataOK(boolean ok) {
      cached_data_ok = ok;  // setter
    }
    char* getName() {
      return(name);
    }
};



// Define a class that extends the ModbusMaster class, but also contains V and I scale factors.
class ModbusMasterScaled: public ModbusMaster{
  private:
    boolean cached_data_ok; 
    float scalei;
    float scalev;
    float scalep;
    char* name;

  public:
    ModbusMasterScaled(char* name):ModbusMaster(), name(name), scalei(1.0), scalev(1.0){
      cached_data_ok = false;
    }
    void setScaleI(float s){
      scalei = s;
    }
    void setScaleV(float s){
      scalev = s;
    }
    float getScaleI(){
      return(scalei);  
    }
    float getScaleV(){
      return(scalev);
    }
    boolean cachedDataOK(){
      return(cached_data_ok);  // getter
    }
    void cachedDataOK(boolean ok){
      cached_data_ok = ok;  // setter
    }
    char* getName(){
      return(name);
    }
};


// Instantiate a ModbusMaster for each Modbus device:
// e.g., Morningstar charge/diversion controllers, Outback inverter, Nuvation BMS
// For each device, appropriate channels must be instantiated - see "Channel Definitions" code below...
// Modbus/RTU devices
ModbusMasterScaled mppt600("mppt600");  // modbus ID=1
ModbusMasterScaled mppt30("mppt30");    // modbus ID=4
ModbusMasterScaled mppt60("mppt60");    // modbus ID=3
ModbusMasterScaled div60("div60");      // modbus ID=2
ModbusMasterScaled div2("div2");        // modbus ID=5

// Modbus/TCP devices
ModbusMasterTCP nuvation("nuvation", parm_nuvation_ip);  // modbus ID=1


class ModbusReg {
  protected:
    int modbus_type;
    ModbusMasterScaled* modbus_dev_ptr;
    ModbusMasterTCP* modbus_dev_ptr_tcp;
    uint8_t modbus_result;  // modbus return code
    char strbuf[30];
    int addr;
    int num_regs;
    int regtype;
    float val_scaled;
    char* chan_name;
    char* chan_label;
    char* units;
    float scale;
    int datatype;
    int strlength;
    union {              // A union is a user-defined type in which all members share the SAME memory location.
      int val_int;       // integer representation= 32-bits, see https://www.arduino.cc/en/Reference/Int
      float val_float;   // float representation = 32-bits, see https://www.arduino.cc/reference/en/language/variables/data-types/float/
      char val_str[20];  // string representation, char = 8-bits, so this is 8*20=160-bits
     };

  public:
    // constructor
    // modbus_dev_ptr: pointer to the Modbus device, e.g., &mppt60
    // regtype: type of register, e.g., MOD_HOLDING_REG
    // addr: address of the channel within the Modbus device
    // chan_name: long name of the channel
    // label: short name of the channel
    // units: string identifying the units, eg. "V"
    // datatype: int describing the data type, from selections above, e.g., MOD_HALFWORD
    // strlength: for string parms specifies its length in chars
    // 
    // For Modbus/RTU devices:
    ModbusReg(ModbusMasterScaled* modbus_dev_ptr, int regtype, int addr, char* chan_name, char* label, char* units, int datatype, int strlength): 
            modbus_dev_ptr(modbus_dev_ptr), regtype(regtype), addr(addr), chan_name(chan_name), chan_label(label), units(units), datatype(datatype),
            strlength(strlength), modbus_result(255){
      setNumRegs();
      modbus_type = MODBUS_TYPE_RTU;
    }
    ModbusReg(ModbusMasterScaled* modbus_dev_ptr, int regtype, int addr, char* chan_name, char * label, char* units, int datatype): 
            modbus_dev_ptr(modbus_dev_ptr), regtype(regtype), addr(addr), chan_name(chan_name), chan_label(label), units(units), datatype(datatype),
            strlength(0), modbus_result(255){
      setNumRegs();
      modbus_type = MODBUS_TYPE_RTU;
    }
    
    // For Modbus/TCP devices:
    ModbusReg(ModbusMasterTCP* modbus_dev_ptr_tcp, int regtype, int addr, char* chan_name, char* label, char* units, int datatype, int strlength): 
            modbus_dev_ptr_tcp(modbus_dev_ptr_tcp), regtype(regtype), addr(addr), chan_name(chan_name), chan_label(label), units(units), datatype(datatype),
            strlength(strlength), modbus_result(255){
      setNumRegs();
      modbus_type = MODBUS_TYPE_TCP;
    }
    ModbusReg(ModbusMasterTCP* modbus_dev_ptr_tcp, int regtype, int addr, char* chan_name, char* label, char* units, int datatype): 
            modbus_dev_ptr_tcp(modbus_dev_ptr_tcp), regtype(regtype), addr(addr), chan_name(chan_name), chan_label(label), units(units), datatype(datatype),
            strlength(0), modbus_result(255){
      setNumRegs();
      modbus_type = MODBUS_TYPE_TCP;
    }


    // readReg(true) assumes a previous long read (of multiple regs) has been saved in a "<device>_cache" object
    // and reads from that object instead of doing a new Modbus access.
    // Data conversions are done as required according to the ModbusReg's data type.
    int readReg() {
      return readReg(false);  // readReg() without arg = readReg(false)
    }
    int readReg(boolean use_cached) {
      int response_buffer_offset = 0;
      static int mppt600_cache_addr = 0x0000;  // readReg(false) sets mppt600_cache_addr=0x0018, subsequent readReg(true) uses this val
      static int mppt60_cache_addr = 0x0000;
      static int mppt30_cache_addr = 0x0000;
      static int div60_cache_addr = 0x0000;
      static int div2_cache_addr = 0x0000;

      // Clear the response buffer for the device ONLY if we're doing a long read! because...
      // We must preserve response buffer data so subsequent calls of readReg(true) can extract the data for individual regs at a known offset.
      if (datatype == MOD_LONG_READ) {
        if (modbus_type == MODBUS_TYPE_RTU) modbus_dev_ptr->clearResponseBuffer();
        if (modbus_type == MODBUS_TYPE_TCP) modbus_dev_ptr_tcp->clearResponseBuffer();
      }

      if (modbus_type == MODBUS_TYPE_RTU) {
        if (use_cached) {                          // if readReg(true)...
          if (modbus_dev_ptr->cachedDataOK()) {    //   if we have cached data...
            //response_buffer_offset = addr & 0x3f;  //     bitwise AND with 0x3f==0011 1111 returns 6 bits of addr, so response_buffer_offset <= 63
                                                   //     This method of setting response_buffer_offset REQUIRES that long reads begin at 
                                                   //     0x0000, 0x0040, 0x0080, 0x00C0, ..., i.e., in groups of 64 regs.
                                                   //     Depending on the addr of the first needed reg, this means that we might have to read many 
                                                   //     regs (before the first needed one) that we don't need!
                                                   //     What we REALLY want is response_buffer_offset = addr - start_addr:
            if (modbus_dev_ptr->getName() == "mppt600") response_buffer_offset = addr - mppt600_cache_addr;  // OVERRIDES above instruction!
            if (modbus_dev_ptr->getName() == "mppt30") response_buffer_offset = addr - mppt30_cache_addr;
            if (modbus_dev_ptr->getName() == "mppt60") response_buffer_offset = addr - mppt60_cache_addr;
            if (modbus_dev_ptr->getName() == "div60") response_buffer_offset = addr - div60_cache_addr;
            if (modbus_dev_ptr->getName() == "div2") response_buffer_offset = addr - div2_cache_addr;
            modbus_result = 0;                     //     mark result successful
          } else {                                 //   otherwise...
            modbus_result = 99;                    //     mark result unsuccessful
          }
        } else {                                   // otherwise, we have readReg(false)...
          if (regtype == MOD_INPUT_REG) {          //   if register type is MOD_INPUT_REG...
            modbus_result = modbus_dev_ptr->readInputRegisters(addr, num_regs);
          } else {                                 //   otherwise, register type is MOD_HOLDING_REG or MOD_NOTHING (for long reads)
            modbus_result = modbus_dev_ptr->readHoldingRegisters(addr, num_regs);
            if ( (modbus_dev_ptr->getName() == "mppt600") && (datatype == MOD_LONG_READ) ) mppt600_cache_addr = addr;  // --> 0x0018
            if ( (modbus_dev_ptr->getName() == "mppt30") && (datatype == MOD_LONG_READ) ) mppt30_cache_addr = addr;
            if ( (modbus_dev_ptr->getName() == "mppt60") && (datatype == MOD_LONG_READ) ) mppt60_cache_addr = addr;
            if ( (modbus_dev_ptr->getName() == "div60") && (datatype == MOD_LONG_READ) ) div60_cache_addr = addr;
            if ( (modbus_dev_ptr->getName() == "div2") && (datatype == MOD_LONG_READ) ) div2_cache_addr = addr;
          }
        }
        //Serial << "modbus.h: modbus_result (RTU) = " << modbus_result << "\n";
      }
      // END if (modbus_type == MODBUS_TYPE_RTU)

      
      // Exactly as above, except modbus_dev_ptr --> modbus_dev_ptr_tcp, and relevant devices change.
      if (modbus_type == MODBUS_TYPE_TCP) {
        modbus_dev_ptr_tcp->updateIPAddress();  // set IPAddress to current parm val

        // Read data from a long read cache object
        if (use_cached) {
          if (modbus_dev_ptr_tcp->cachedDataOK()) {
            response_buffer_offset = addr & 0x3f;
            modbus_result = 0;
          } else {
            modbus_result = 99;
          }
        } else {
          if (regtype == MOD_INPUT_REG) {
            modbus_result = modbus_dev_ptr_tcp->readInputRegisters(addr, num_regs);  
          } else {
            modbus_result = modbus_dev_ptr_tcp->readHoldingRegisters(addr, num_regs);
          }
        }
       //Serial << "modbus.h: modbus_result (TCP) = " << modbus_result << "\n";
      }
      // END if (modbus_type == MODBUS_TYPE_TCP)


      // DEBUG: Specify some restriction in the if() on what to print - too much otherwise!
      if (false) {
        Serial << "modbus.h: readReg() addr=" << addr << ", offset=" << response_buffer_offset << ", chan_name=" << chan_name << ", datatype=" << datatype 
               << ", val=" << modbus_dev_ptr->getResponseBuffer(response_buffer_offset) << ", result=" << getErrorStrg() << "\n";
      }
      // DEBUG: if this is a long read, print entire response buffer
      if (false) {
      //if ( (datatype == MOD_LONG_READ) && (chan_name == "MPPT60 long read") ) {
        char buf[10];
        for (int i = 0; i < 63; i++) {
          sprintf(buf, "%X", i);
          if (modbus_type == MODBUS_TYPE_RTU) Serial << buf << " " << modbus_dev_ptr->getResponseBuffer(i) << "\n";
          if (modbus_type == MODBUS_TYPE_TCP) Serial << buf << " " << modbus_dev_ptr_tcp->getResponseBuffer(i) << "\n";
        }
      }
      // END DEBUG
        
      
      // get first halfword = 16 bits = contents of 1 Modbus register
      if (modbus_result == 0) {                                                     // if Modbus read is successful...
        if (modbus_type == MODBUS_TYPE_RTU) {                                       //   if modbus_type is RTU...
          if (datatype == MOD_LONG_READ) modbus_dev_ptr->cachedDataOK(true);        //     if this is a long read, mark it so
          val_int = modbus_dev_ptr->getResponseBuffer(response_buffer_offset);      //     get buffer contents at addr + offset
        }
        if (modbus_type == MODBUS_TYPE_TCP) {                                       //    if modbus_type is TCP...
          if (datatype == MOD_LONG_READ) modbus_dev_ptr_tcp->cachedDataOK(true);    //     if this is a long read, mark it so
          val_int = modbus_dev_ptr_tcp->getResponseBuffer(response_buffer_offset);  //     get buffer contents at addr + offset
        }
        
        switch (datatype) {
          case MOD_HALFWORD:  // 16 bits: 0x0000 - 0xffff
            break;
          case MOD_HALFWORD_SIGNED:  // sign extend
            if (val_int & 0x8000) val_int |= 0xffff0000;  // if bit 1000 0000 0000 0000 is set, do a bitwise OR extending to 32-bits
            break;
          case MOD_HALFWORD_SIGNED_SCALED:  // added for Nuvation regs which contain signed int's that need scaling (aw)
            if (val_int & 0x8000) val_int |= 0xffff0000;
            val_float = val_int * scale;
            break;
          case MOD_FULLWORD:  // 32 bits: 0x00000000 - 0xffffffff
          case MOD_FLOAT32:
            if (modbus_type == MODBUS_TYPE_RTU) val_int = (val_int << 16) + modbus_dev_ptr->getResponseBuffer(response_buffer_offset + 1);
            if (modbus_type == MODBUS_TYPE_TCP) val_int = (val_int << 16) + modbus_dev_ptr_tcp->getResponseBuffer(response_buffer_offset + 1);
            //Serial << "modbus.h: " << "MOD_FULLWORD " << val_int << " " << response_buffer_offset << "\n";
            break;
          case MOD_FLOAT16:  // convert a signed 16-bit float to a signed 32-bit value
            // 16-bit float:
            // SEEEEEFFFFFFFFFF   sign, 5 exponent bits, 10 fraction bits
            // 32-bit float (IEEE 754/binary32 standard):
            // SEEEEEEEEFFFFFFFFFFFFFFFFFFFFFFF sign, 8 exponent, 23 fraction
            // See https://forum.arduino.cc/t/understanding-32-bit-floating-point-number-representation-binary32-format/691221
            // Aligned:
            // S   EEEEEFFFFFFFFFF
            // SEEEEEEEEFFFFFFFFFFFFFFFFFFFFFFF
            // 10987654321098765432109876543210
            //  3         2         1
            // Sign bit: Bit 15 of the F16 --> bit 31 the F32.
            // Fraction bits: Bits 0-9 of the F16 --> bits 13-22 of the F32.
            // Exponent bits: Bits 10-14 of the F16 --> bits 23-27 of the F32.
            // The exponent is stored in "offset binary" representation.
            // For the 16-bit float, the stored exponent is the actual exponent plus 15.
            // For the 32-bit float, the stored exponent is the actual exponent plus 127.
            // Mathematically, Sto(16) = Act + 15 --> Act = Sto(16) - 15 and Sto(32) = Act + 127, so Sto(32) = Sto(16) - 15 + 127 = Sto(16) + 112
            // So, we need to add 112 to the F16 exponent to give it an offset 127 for the F32.
            //   1. Shift F16 sign bit left 16 (bit 15 to bit 31),
            //   2. add 112 to F16 exponent,
            //   3. shift F16 exponent and fraction left 13 (bits 0-14 to bits 13-27),
            //   0x8000 =       1000 0000 0000 0000 (selects sign bit of 16-bit int)
            //   0x7fff =       0111 1111 1111 1111 (selects non-sign bits of 16-bit int)
            //   112 (base 10) = 111 0000 << 10 (skip over the 10 fraction bits into the F16 exponent) gives 1 1100 0000 0000 0000 = 0x1c000
            val_int = ((val_int & 0x8000) << 16) | (((val_int & 0x7fff) + 0x1c000) << 13);
            break;
          case MOD_SCALED_V:
            //Serial << val_int << " " << modbus_dev_ptr->getScaleV() << "\n";
            // if MSB is 1 in input 16-bit signed int, then preserve all the bits in the input signed int and put 1's into bits 16-31 of Due 32-bit int
            if (val_int & 0x8000) val_int = 0xffff0000 | val_int;
            if (modbus_type == MODBUS_TYPE_RTU) val_float = val_int * modbus_dev_ptr->getScaleV() / 32768.0;  // 32768 = 2^15
            if (modbus_type == MODBUS_TYPE_TCP) val_float = val_int * modbus_dev_ptr_tcp->getScaleV() / 32768.0;  // 32768 = 2^15
            break;
          case MOD_SCALED_I:
            // if MSB is 1 in input 16-bit signed int, then preserve all the bits in the input signed int and put 1's into bits 16-31 of Due 32-bit int
            if (val_int & 0x8000) val_int = 0xffff0000 | val_int;
            if (modbus_type == MODBUS_TYPE_RTU) val_float = val_int * modbus_dev_ptr->getScaleI() / 32768.0;  // 32768 = 2^15
            if (modbus_type == MODBUS_TYPE_TCP) val_float = val_int * modbus_dev_ptr_tcp->getScaleI() / 32768.0;  // 32768 = 2^15
            break;
          case MOD_SCALED_P:
            if (modbus_type == MODBUS_TYPE_RTU) val_float = val_int * modbus_dev_ptr->getScaleV() * modbus_dev_ptr->getScaleI() / 131072.0;  // 131072 = 2^17
            if (modbus_type == MODBUS_TYPE_TCP) val_float = val_int * modbus_dev_ptr_tcp->getScaleV() * modbus_dev_ptr_tcp->getScaleI() / 131072.0;  // 131072 = 2^17
            break;
          case MOD_SCALED:
            val_float = val_int * scale;
            break;            
        }

        // Put result of datatype manipulations above into a string buffer which is returned when method ->valStrg() is called.
        switch (datatype) {
          case MOD_HALFWORD:
          case MOD_HALFWORD_SIGNED:
          case MOD_FULLWORD:
            sprintf(strbuf, "%d", val_int);
            break;
          case MOD_FLOAT16:
          case MOD_FLOAT32:
          case MOD_SCALED_V:
          case MOD_SCALED_I:
          case MOD_SCALED_P:
          case MOD_SCALED:
          case MOD_HALFWORD_SIGNED_SCALED:
            sprintf(strbuf, "%8.3f", val_float);
            break;
        }
      } else {  // modbus_result !== 0, bad read
        strcpy(strbuf, "\"NaN\"");
        // A bad long read sets a flag that is consulted on every subsequent cached read,
        // leads to setting modbus_result = 99 on these reads, and outputting of "NaN".
        if (datatype == MOD_LONG_READ) {
          if (modbus_type == MODBUS_TYPE_RTU) {
            modbus_dev_ptr->cachedDataOK(false);  // marks bad read
            Serial << "modbus.h: " << modbus_dev_ptr->getName() << ": Non-zero return code on long read: " << getErrorStrg() << "\n";
          }
          if (modbus_type == MODBUS_TYPE_TCP) {
            modbus_dev_ptr_tcp->cachedDataOK(false);  // marks bad read
            Serial << "modbus.h: " << modbus_dev_ptr_tcp->getName() << ": Non-zero return code on long read: " << getErrorStrg() << "\n";
          }
        }
      }
      return modbus_result;
    }  // END int readReg(boolean use_cached)
  
    int valInt(){
      return val_int;
    }
    
    float valFloat(){
      return val_float;
    }
    
    char* valStrg(){
      return(strbuf);
    }
    
    char* getChanName(){
      return(chan_name);
    }
    
    char* getChanLabel(){
      return(chan_label);
    }
    
    char* getUnits(){
      return(units);
    }
    
    void setScale(float s){
      scale = s;
    }

    void setNumRegs(int n) {  // setter
      num_regs = n;
    }

    void setNumRegs() {  // getter
      switch (datatype) {
        case MOD_HALFWORD:
        case MOD_HALFWORD_SIGNED:
        case MOD_HALFWORD_SIGNED_SCALED:
        case MOD_FLOAT16:
        case MOD_SCALED_V:
        case MOD_SCALED_I:
        case MOD_SCALED_P:
        case MOD_SCALED:
          num_regs = 1;
          break;
        case MOD_FULLWORD:
        case MOD_FLOAT32:
          num_regs = 2;
          break;
        case MOD_STRING:
          num_regs = strlength / 2;
          break;
        case MOD_LONG_READ:
          num_regs= 64;
          break;
      }
    }
    
    char* getErrorStrg() {
      switch (modbus_result) {
        case 0x00:
          return "success";
          break;
        case 0x01:
          return "ERROR: Illegal Function";
          break;
        case 0x02:
          return "ERROR: Illegal Data Address";
          break;
        case 0x03:
          return "ERROR: Illegal Data Value";
          break;
        case  0x04:
          return "ERROR: Device Failure";
          break;
        case 0xe0:
          return "ERROR: Invalid Slave ID";
          break;
        case 0xe1:
          return "ERROR: Invalid Function";
          break;
        case 0xe2:
          return "ERROR: Response Timeout";
          break;
        case 0xe3:
          return "ERROR: CRC Error";
          break;
        case 255:
          return "ERROR: Uninitialized";
          break;
        case 99:
          return "ERROR: on cache access previous long read";
          break;
      }
    }
};  // END class ModbusReg{}




//-------------------------------------------------------------------------
// Channel Definitions
// Create instances of individual channels accessed from each Modbus device.
// ModbusMaster(<modbus_dev_ptr>, int <reg type>, int <addr>, char* <chan_name>, char* <chan_name_for_udp>, char* <units>, int <datatype>, int <strlength>
// NOTE: the last parm is optional, only used for strings.
//
// *** TO ADD A CHANNEL ***
//   1. Create an instance of the desired channel in one of the lists of ModbusReg class objects below
//   2. Add the new channel to mod_nuv_regs[] or mod_fast_regs[] or mod_slow_regs[] - found after the ModbusReg class lists
//   3. If the channel is to be included in a long read cache, change the <device>_cache.setNumRegs(#) in modbus.ino
//-------------------------------------------------------------------------
// Nuvation low-voltage BMS (16-bit registers)
ModbusReg nuvation_Vol = ModbusReg(&nuvation, MOD_HOLDING_REG, 40105, "Batt stack V", "Nuv_Vol", "V", MOD_SCALED);
ModbusReg nuvation_MaxBatACha = ModbusReg(&nuvation, MOD_HOLDING_REG, 40106, "Batt max charge I", "Nuv_MaxBatACha", "A", MOD_SCALED);
ModbusReg nuvation_MaxBatADischa = ModbusReg(&nuvation, MOD_HOLDING_REG, 40107, "Batt max discharge I", "Nuv_MaxBatADischa", "A", MOD_SCALED);
ModbusReg nuvation_Vol_SF = ModbusReg(&nuvation, MOD_HOLDING_REG, 40113, "Batt stack V SF", "Nuv_Vol_SF", "", MOD_HALFWORD_SIGNED);
ModbusReg nuvation_MaxBatA_SF = ModbusReg(&nuvation, MOD_HOLDING_REG, 40114, "Batt max I SF", "Nuv_MaxBatA_SF", "", MOD_HALFWORD_SIGNED);
ModbusReg nuvation_BMaxCellVol = ModbusReg(&nuvation, MOD_HOLDING_REG, 40119, "Batt max cell V", "Nuv_BMaxCellVol", "V", MOD_SCALED);
ModbusReg nuvation_BMinCellVol = ModbusReg(&nuvation, MOD_HOLDING_REG, 40121, "Batt min cell V", "Nuv_BMinCellVol", "V", MOD_SCALED);
ModbusReg nuvation_BMaxModTemp = ModbusReg(&nuvation, MOD_HOLDING_REG, 40123, "Batt max cell T", "Nuv_BMaxModTemp", "C", MOD_HALFWORD_SIGNED_SCALED);
ModbusReg nuvation_BMinModTemp = ModbusReg(&nuvation, MOD_HOLDING_REG, 40125, "Batt min cell T", "Nuv_BMinModTemp", "C", MOD_HALFWORD_SIGNED_SCALED);
ModbusReg nuvation_BTotDCCurr = ModbusReg(&nuvation, MOD_HOLDING_REG, 40127, "Batt total DC I", "Nuv_BTotDCCurr", "A", MOD_HALFWORD_SIGNED_SCALED);
ModbusReg nuvation_BCellVol_SF = ModbusReg(&nuvation, MOD_HOLDING_REG, 40130, "Batt cell V SF", "Nuv_BCellVol_SF", "", MOD_HALFWORD_SIGNED);
ModbusReg nuvation_BModTemp_SF = ModbusReg(&nuvation, MOD_HOLDING_REG, 40131, "Batt cell T SF", "Nuv_BModTemp_SF", "", MOD_HALFWORD_SIGNED);
ModbusReg nuvation_BCurrent_SF = ModbusReg(&nuvation, MOD_HOLDING_REG, 40132, "Batt I SF", "Nuv_BCurrent_SF", "", MOD_HALFWORD_SIGNED);


// Morningstar TS-MPPT-600V controller (WIND)
// IMPORTANT: The setNumRegs() method is used in modbus.ino to set the # of CONTIGUOUS regs for a long read. Adjust if changes are made here!
// LONG READ REGISTER RANGE = 0x0018 to 0x0044 --> 68 - 24 + 1 = 45 registers
ModbusReg mppt600_cache = ModbusReg(&mppt600, MOD_NOTHING, 0x0018, "MPPT600 long read", "", "", MOD_LONG_READ);

// RAM (holding) registers - uncomment only those that are in use!
ModbusReg mppt600_ver_sw = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x0004, "MPPT600 software version", "sw600", "", MOD_HALFWORD);  // 16-bit int
ModbusReg mppt600_adc_vb_f_med = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x0018, "MPPT600 battery voltage, filtered",  "Vb600", "V",  MOD_FLOAT16);
ModbusReg mppt600_adc_va_f_shadow = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x001B, "MPPT600 wind voltage, filtered",  "Vw600", "V", MOD_FLOAT16);
ModbusReg mppt600_adc_ib_f_shadow = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x001C, "MPPT600 battery current, filtered", "Ib600", "A", MOD_FLOAT16);
ModbusReg mppt600_adc_ia_f_shadow = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x001D, "MPPT600 wind current, filtered", "Iw600", "A", MOD_FLOAT16);
ModbusReg mppt600_T_hs = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x0023, "MPPT600 heatsink temperature", "Ths600", "C", MOD_FLOAT16);
ModbusReg mppt600_fault_i = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x002C, "MPPT600 fault", "FB600", "", MOD_FULLWORD);  // 32-bits, 2 regs!
ModbusReg mppt600_alarm_i = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x002E, "MPPT600 alarm", "AB600", "", MOD_FULLWORD);  // 32-bits, 2 regs!
ModbusReg mppt600_mb_charge_state = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x0032, "MPPT600 charging state", "state600", "", MOD_HALFWORD);  // 16-bit int
ModbusReg mppt600_vb_ref = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x0033, "MPPT600 Target regulation voltage", "Vbref600", "V", MOD_FLOAT16);
ModbusReg mppt600_Ahc_r = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x0034, "MPPT600 Ah resettable", "Ah600", "Ah", MOD_FLOAT32);  // 32-bits, 2 regs!
ModbusReg mppt600_kwhc_r = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x0038, "MPPT600 kWh resettable", "kWh600", "kWh", MOD_FLOAT16);  // updated at ~0.25 kW intervals, NOT USEFUL
// The following register is *not* MOD_HALFWORD as per Morningstar documentation! It is MOD_FLOAT16 with units of kWh and 3-decimal precision (1 Wh).
// Since this controller is always in MPPT mode, NIGHT mode never occurs (which is the reset method for Whc_daily in the TS-MPPT-60 and -30 controllers below).
// ***mppt600_Whc_daily is reset every 24h of controller runtime (after a power off/on reset).***
ModbusReg mppt600_Whc_daily = ModbusReg(&mppt600, MOD_HOLDING_REG, 0x0044, "MPPT600 kWh daily", "Wh600", "kWh", MOD_FLOAT16);

// EEPROM (input) registers - uncomment only those that are in use!
/*
ModbusReg mppt600_EV_absorb = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE000, "MPPT600 absorption voltage @25C", "absorbV600", "V", MOD_FLOAT16);
ModbusReg mppt600_EV_float = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE001, "MPPT600 float voltage @25C", "", "floatV600", MOD_FLOAT16);
ModbusReg mppt600_Et_absorp = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE002, "MPPT600 absorption time", "absorbT600", "s", MOD_HALFWORD);
ModbusReg mppt600_EV_eq = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE007, "MPPT600 equalize voltage @25C", "equalV600", "V", MOD_FLOAT16);
ModbusReg mppt600_Emodbus_id = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE019, "MPPT600 modbus slave address", "modAddr600", "", MOD_HALFWORD);
*/
ModbusReg mppt600_PV_P_0 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE023, "MPPT600 P/V Curve P 0", "PV_P_0", "W", MOD_FLOAT16);  // wind P/V curve: POWER registers
ModbusReg mppt600_PV_P_1 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE024, "MPPT600 P/V Curve P 1", "PV_P_1", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_2 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE025, "MPPT600 P/V Curve P 2", "PV_P_2", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_3 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE026, "MPPT600 P/V Curve P 3", "PV_P_3", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_4 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE027, "MPPT600 P/V Curve P 4", "PV_P_4", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_5 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE028, "MPPT600 P/V Curve P 5", "PV_P_5", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_6 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE029, "MPPT600 P/V Curve P 6", "PV_P_6", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_7 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE02A, "MPPT600 P/V Curve P 7", "PV_P_7", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_8 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE02B, "MPPT600 P/V Curve P 8", "PV_P_8", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_9 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE02C, "MPPT600 P/V Curve P 9", "PV_P_9", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_10 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE02D, "MPPT600 P/V Curve P 10", "PV_P_10", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_11 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE02E, "MPPT600 P/V Curve P 11", "PV_P_11", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_12 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE02F, "MPPT600 P/V Curve P 12", "PV_P_12", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_13 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE030, "MPPT600 P/V Curve P 13", "PV_P_13", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_14 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE031, "MPPT600 P/V Curve P 14", "PV_P_14", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_P_15 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE032, "MPPT600 P/V Curve P 15", "PV_P_15", "W", MOD_FLOAT16);
ModbusReg mppt600_PV_V_0 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE033, "MPPT600 P/V Curve V 0", "PV_V_0", "V", MOD_FLOAT16);  // wind P/V curve: VOLTAGE registers
ModbusReg mppt600_PV_V_1 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE034, "MPPT600 P/V Curve V 1", "PV_V_1", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_2 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE035, "MPPT600 P/V Curve V 2", "PV_V_2", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_3 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE036, "MPPT600 P/V Curve V 3", "PV_V_3", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_4 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE037, "MPPT600 P/V Curve V 4", "PV_V_4", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_5 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE038, "MPPT600 P/V Curve V 5", "PV_V_5", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_6 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE039, "MPPT600 P/V Curve V 6", "PV_V_6", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_7 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE03A, "MPPT600 P/V Curve V 7", "PV_V_7", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_8 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE03B, "MPPT600 P/V Curve V 8", "PV_V_8", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_9 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE03C, "MPPT600 P/V Curve V 9", "PV_V_9", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_10 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE03D, "MPPT600 P/V Curve V 10", "PV_V_10", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_11 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE03E, "MPPT600 P/V Curve V 11", "PV_V_11", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_12 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE03F, "MPPT600 P/V Curve V 12", "PV_V_12", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_13 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE040, "MPPT600 P/V Curve V 13", "PV_V_13", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_14 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE041, "MPPT600 P/V Curve V 14", "PV_V_14", "V", MOD_FLOAT16);
ModbusReg mppt600_PV_V_15 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE042, "MPPT600 P/V Curve V 15", "PV_V_15", "V", MOD_FLOAT16);
ModbusReg mppt600_Eserial0 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE0C0, "MPPT600 SN0", "sn0_600", "", MOD_HALFWORD);  // 2 ASCII chars (of 8 total)
ModbusReg mppt600_Eserial1 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE0C1, "MPPT600 SN1", "sn1_600", "", MOD_HALFWORD);
ModbusReg mppt600_Eserial2 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE0C2, "MPPT600 SN2", "sn2_600", "", MOD_HALFWORD);
ModbusReg mppt600_Eserial3 = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE0C3, "MPPT600 SN3", "sn3_600", "", MOD_HALFWORD);
ModbusReg mppt600_Ehw_version = ModbusReg(&mppt600, MOD_INPUT_REG, 0xE0CD, "MPPT600 HW ver", "hwVer600", "", MOD_HALFWORD); // byte1=major, byte0=minor


// Morningstar TS-MPPT-30 controller (PV1)
// IMPORTANT: The setNumRegs() method is used in modbus.ino to set the # of CONTIGUOUS regs for a long read. Adjust if changes are made here!
// LONG READ REGISTER RANGE = 0x0018 to 0x0044 --> 68 - 24 + 1 = 45 registers
ModbusReg mppt30_cache = ModbusReg(&mppt30, MOD_NOTHING, 0x0018, "MPPT30 long read", "", "", MOD_LONG_READ);

// RAM (holding) registers:
ModbusReg mppt30_V_PU = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x0000, "MPPT30 V scale whole", "V_PU_hi", "", MOD_FULLWORD);
ModbusReg mppt30_I_PU = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x0002, "MPPT30 I scale whole", "I_PU_hi", "", MOD_FULLWORD);
ModbusReg mppt30_ver_sw = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x0004, "MPPT30 software version", "sw30", "", MOD_HALFWORD);
ModbusReg mppt30_adc_vb_f_med = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x0018, "MPPT30 batt voltage, filt.", "Vb30", "V", MOD_SCALED_V);
ModbusReg mppt30_adc_va = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x001B, "MPPT30 array voltage, filt.", "Va30", "V", MOD_SCALED_V);
ModbusReg mppt30_adc_ib_f_shadow = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x001C, "MPPT30 batt current, filt.", "Ib30", "A", MOD_SCALED_I);
ModbusReg mppt30_adc_ia_f_shadow = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x001D, "MPPT30 array current, filt.", "Ia30", "A", MOD_SCALED_I);
ModbusReg mppt30_T_hs = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x0023, "MPPT30 heatsink temp", "Ths30", "C", MOD_HALFWORD_SIGNED);
ModbusReg mppt30_fault = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x002C, "MPPT30 fault", "FB30", "", MOD_HALFWORD);
ModbusReg mppt30_alarm = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x002E, "MPPT30 alarm", "AB30", "", MOD_FULLWORD);  // read 2 registers
ModbusReg mppt30_charge_state = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x0032, "MPPT30 charging stage", "state30", "", MOD_HALFWORD);
ModbusReg mppt30_vb_ref = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x0033, "MPPT30 target regulation voltage", "Vbref30", "V", MOD_SCALED_V);
ModbusReg mppt30_Ahc_r = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x0035, "MPPT30 Ah resettable", "Ah30", "Ah", MOD_SCALED);   // scale: n*0.1 (read LO word only)
ModbusReg mppt30_whc_daily = ModbusReg(&mppt30, MOD_HOLDING_REG, 0x0044, "MPPT30 Wh daily", "Wh30",  "Wh", MOD_HALFWORD);  // 

// EEPROM (input) registers - uncomment only those that are in use!
//ModbusReg mppt30_EV_absorb = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE000, "MPPT30 Absorption voltage @25C", "absorbV30", "V", MOD_FLOAT16);
//ModbusReg mppt30_EV_float = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE001, "MPPT30 Float voltage @25C", "floatV30", "V", MOD_FLOAT16);
//ModbusReg mppt30_EV_eq = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE007, "MPPT30 Equalize voltage @25C", "equalV30", "V", MOD_FLOAT16);
ModbusReg mppt30_EV_hvd = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE00E, "MPPT30 Battery High Voltage Disconnect", "hvd30", "V", MOD_SCALED_V);
ModbusReg mppt30_EV_hvr = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE00F, "MPPT30 Battery High Voltage Reconnect", "hvr30", "V", MOD_SCALED_V);
//ModbusReg mppt30_Emodbus_id = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE019, "MPPT30 MODBUS slave address", "modAddr30", "", MOD_HALFWORD);
ModbusReg mppt30_Eserial0 = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE0C0, "MPPT30 Serial Number", "sn0_30", "", MOD_HALFWORD);
ModbusReg mppt30_Eserial1 = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE0C1, "MPPT30 Serial Number", "sn1_30", "", MOD_HALFWORD);
ModbusReg mppt30_Eserial2 = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE0C2, "MPPT30 Serial Number", "sn2_30", "", MOD_HALFWORD);
ModbusReg mppt30_Eserial3 = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE0C3, "MPPT30 Serial Number", "sn3_30", "", MOD_HALFWORD);
ModbusReg mppt30_Ehw_version = ModbusReg(&mppt30, MOD_INPUT_REG, 0xE0CD, "MPPT30 Hardware version, vMajor.Minor", "hwVer30", "", MOD_HALFWORD);


// Morningstar TS-MPPT-60 controller (PV2)
// IMPORTANT: The setNumRegs() method is used in modbus.ino to set the # of CONTIGUOUS regs for a long read. Adjust if changes are made here!
// LONG READ REGISTER RANGE = 0x0018 to 0x0044 --> 68 - 24 + 1 = 45 registers
ModbusReg mppt60_cache = ModbusReg(&mppt60, MOD_NOTHING, 0x0018, "MPPT60 long read", "", "", MOD_LONG_READ);

// RAM (holding) registers - uncomment only those that are in use!
ModbusReg mppt60_V_PU = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x0000, "MPPT60 V scale", "V_PU_hi", "", MOD_FULLWORD);
ModbusReg mppt60_I_PU = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x0002, "MPPT60 I scale", "I_PU_hi", "", MOD_FULLWORD);
ModbusReg mppt60_ver_sw = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x0004, "MPPT60 software version", "sw60", "", MOD_HALFWORD);
ModbusReg mppt60_adc_vb_f_med = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x0018, "MPPT60 batt voltage, filt.", "Vb60", "V", MOD_SCALED_V);
ModbusReg mppt60_adc_va = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x001B, "MPPT60 array voltage, filt.", "Va60", "V", MOD_SCALED_V);
ModbusReg mppt60_adc_ib_f_shadow = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x001C, "MPPT60 batt current, filt.", "Ib60", "A", MOD_SCALED_I);
ModbusReg mppt60_adc_ia_f_shadow = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x001D, "MPPT60 array current, filt.", "Ia60", "A", MOD_SCALED_I);
ModbusReg mppt60_T_hs = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x0023, "MPPT60 heatsink temp", "Ths60", "C", MOD_HALFWORD_SIGNED);
ModbusReg mppt60_fault = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x002C, "MPPT60 fault", "FB60", "", MOD_HALFWORD);
ModbusReg mppt60_alarm = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x002E, "MPPT60 alarm", "AB60", "", MOD_FULLWORD);  // read 0x002E and 0x002F regs together
ModbusReg mppt60_charge_state = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x0032, "MPPT60 charging stage", "state60", "", MOD_HALFWORD);
ModbusReg mppt60_vb_ref = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x0033, "MPPT60 target regulation voltage", "Vbref60", "V", MOD_SCALED_V);
ModbusReg mppt60_Ahc_r = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x0035, "MPPT60 Ah resettable", "Ah60",  "Ah", MOD_SCALED);  // n*0.1 (read LO word only)
ModbusReg mppt60_whc_daily = ModbusReg(&mppt60, MOD_HOLDING_REG, 0x0044, "MPPT60 Wh daily", "Wh60",  "Wh", MOD_HALFWORD);  // 

// EEPROM (input) registers - uncomment only those that are in use!
//ModbusReg mppt60_EV_absorb = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE000, "MPPT60 Absorption voltage @25C", "absorbV60", "V", MOD_FLOAT16);
//ModbusReg mppt60_EV_float = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE001, "MPPT60 Float voltage @25C", "floatV60", "V", MOD_FLOAT16);
//ModbusReg mppt60_EV_eq = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE007, "MPPT60 Equalize voltage @25C", "equalV60", "V", MOD_FLOAT16);
ModbusReg mppt60_EV_hvd = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE00E, "MPPT60 Battery High Voltage Disconnect", "hvd60", "V", MOD_SCALED_V);
ModbusReg mppt60_EV_hvr = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE00F, "MPPT60 Battery High Voltage Reconnect", "hvr60", "V", MOD_SCALED_V);
//ModbusReg mppt60_Emodbus_id = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE019, "MPPT60 MODBUS slave address", "modAddr60", "", MOD_HALFWORD);
ModbusReg mppt60_Eserial0 = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE0C0, "MPPT60 Serial Number", "sn0_60", "", MOD_HALFWORD);
ModbusReg mppt60_Eserial1 = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE0C1, "MPPT60 Serial Number", "sn1_60", "", MOD_HALFWORD);
ModbusReg mppt60_Eserial2 = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE0C2, "MPPT60 Serial Number", "sn2_60", "", MOD_HALFWORD);
ModbusReg mppt60_Eserial3 = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE0C3, "MPPT60 Serial Number", "sn3_60", "", MOD_HALFWORD);
ModbusReg mppt60_Ehw_version = ModbusReg(&mppt60, MOD_INPUT_REG, 0xE0CD, "MPPT60 Hardware version, vMajor.Minor", "hwVer60", "", MOD_HALFWORD);


// Morningstar TS-60 controller (DIV1)
// IMPORTANT: The setNumRegs() method is used in modbus.ino to set the # of CONTIGUOUS regs for a long read. Adjust if changes are made here!
// LONG READ REGISTER RANGE = 0x0008 to 0x001D --> 29 - 8 + 1 = 22 registers
ModbusReg div60_cache = ModbusReg(&div60, MOD_NOTHING, 0x0008, "DIV60 long read", "", "", MOD_LONG_READ);

// RAM (holding) registers
ModbusReg div60_adc_vb_f = ModbusReg(&div60, MOD_HOLDING_REG, 0x0008, "TS60 battery voltage, 2.5s filt.", "VbD60", "V", MOD_SCALED);     // n*96.667*2^-15
ModbusReg div60_adc_vx_f = ModbusReg(&div60, MOD_HOLDING_REG, 0x000A, "TS60 load voltage, 2.5s filt.", "VloadD60", "V", MOD_SCALED);     // n*139.15*2^-15
ModbusReg div60_adc_ipv_f = ModbusReg(&div60, MOD_HOLDING_REG, 0x000B, "TS60 charge current, 2.5s filt.", "IpvD60", "V", MOD_SCALED);    // n*66.667*2^-15
ModbusReg div60_adc_iload_f = ModbusReg(&div60, MOD_HOLDING_REG, 0x000C, "TS60 load current, 2.5s filt.", "IloadD60", "V", MOD_SCALED);  // n*316.67*2^-15
ModbusReg div60_T_hs = ModbusReg(&div60, MOD_HOLDING_REG, 0x000E, "TS60 heatsink temperature", "ThsD60", "C", MOD_HALFWORD_SIGNED);      // -128 to +127
ModbusReg div60_V_ref = ModbusReg(&div60, MOD_HOLDING_REG, 0x0010, "TS60 T-compensated target voltage", "VrefD60", "V", MOD_SCALED);     // n*96.667*2^-15
ModbusReg div60_Ah_r = ModbusReg(&div60, MOD_HOLDING_REG, 0x0012, "TS60 Ah resettable", "AhD60", "Ah", MOD_SCALED);                      // n*0.1 (read LO word only)
ModbusReg div60_Alarm_LO = ModbusReg(&div60, MOD_HOLDING_REG, 0x0017, "TS60 alarm, LO", "ABloD60", "", MOD_HALFWORD);                    // bitfield
ModbusReg div60_fault = ModbusReg(&div60, MOD_HOLDING_REG, 0x0018, "TS60 fault", "FBD60", "", MOD_HALFWORD);                             // bitfield   
ModbusReg div60_control_state = ModbusReg(&div60, MOD_HOLDING_REG, 0x001B, "TS60 control state", "stateD60", "", MOD_HALFWORD);          // 0-8   
ModbusReg div60_d_filt = ModbusReg(&div60, MOD_HOLDING_REG, 0x001C, "TS60 PWM duty cycle", "PWMD60", "", MOD_SCALED);                    // 0-255, >=230=100%
ModbusReg div60_Alarm_HI = ModbusReg(&div60, MOD_HOLDING_REG, 0x001D, "TS60 alarm, HI", "ABhiD60", "", MOD_HALFWORD);                    // bitfield


// Morningstar TS-60 controller (DIV2)
// IMPORTANT: The setNumRegs() method is used in modbus.ino to set the # of CONTIGUOUS regs for a long read. Adjust if changes are made here!
// LONG READ REGISTER RANGE = 0x0008 to 0x001D --> 29 - 8 + 1 = 22 registers
ModbusReg div2_cache = ModbusReg(&div2, MOD_NOTHING, 0x0008, "DIV2 long read", "", "", MOD_LONG_READ);

// RAM (holding) registers
ModbusReg div2_adc_vb_f = ModbusReg(&div2, MOD_HOLDING_REG, 0x0008, "TS60 battery voltage, 2.5s filt.", "VbD2", "V", MOD_SCALED);     // n*96.667*2^-15
ModbusReg div2_adc_vx_f = ModbusReg(&div2, MOD_HOLDING_REG, 0x000A, "TS60 load voltage, 2.5s filt.", "VloadD2", "V", MOD_SCALED);     // n*139.15*2^-15
ModbusReg div2_adc_ipv_f = ModbusReg(&div2, MOD_HOLDING_REG, 0x000B, "TS60 charge current, 2.5s filt.", "IpvD2", "V", MOD_SCALED);    // n*66.667*2^-15
ModbusReg div2_adc_iload_f = ModbusReg(&div2, MOD_HOLDING_REG, 0x000C, "TS60 load current, 2.5s filt.", "IloadD2", "V", MOD_SCALED);  // n*316.67*2^-15
ModbusReg div2_T_hs = ModbusReg(&div2, MOD_HOLDING_REG, 0x000E, "TS60 heatsink temperature", "ThsD2", "C", MOD_HALFWORD_SIGNED);      // -128 to +127
ModbusReg div2_V_ref = ModbusReg(&div2, MOD_HOLDING_REG, 0x0010, "TS60 T-compensated target voltage", "VrefD2", "V", MOD_SCALED);     // n*96.667*2^-15
ModbusReg div2_Ah_r = ModbusReg(&div2, MOD_HOLDING_REG, 0x0012, "TS60 Ah resettable", "AhD2", "Ah", MOD_SCALED);                      // n*0.1 (read LO word only)
ModbusReg div2_Alarm_LO = ModbusReg(&div2, MOD_HOLDING_REG, 0x0017, "TS60 alarm, LO", "ABloD2", "", MOD_HALFWORD);                    // bitfield
ModbusReg div2_fault = ModbusReg(&div2, MOD_HOLDING_REG, 0x0018, "TS60 fault", "FBD2", "", MOD_HALFWORD);                             // bitfield   
ModbusReg div2_control_state = ModbusReg(&div2, MOD_HOLDING_REG, 0x001B, "TS60 control state", "stateD2", "", MOD_HALFWORD);          // 0-8   
ModbusReg div2_d_filt = ModbusReg(&div2, MOD_HOLDING_REG, 0x001C, "TS60 PWM duty cycle", "PWMD2", "", MOD_SCALED);                    // 0-255, >=230=100%
ModbusReg div2_Alarm_HI = ModbusReg(&div2, MOD_HOLDING_REG, 0x001D, "TS60 alarm, HI", "ABhiD2", "", MOD_HALFWORD);                    // bitfield


// Populate the arrays below with Modbus register names.
// These arrays specify which channels are to be read every second and sent out in UDP packets.
// Do not include scale factors, as they are read once only (during Modbus init).

const int NUM_MOD_NUV_CHANNELS = 8;  // MOD_NUM_NUV_REGS must = # channels in the array below
ModbusReg* mod_nuv_regs[] = {
  &nuvation_Vol,
  &nuvation_MaxBatACha,
  &nuvation_MaxBatADischa,
  &nuvation_BMaxCellVol,
  &nuvation_BMinCellVol,
  &nuvation_BMaxModTemp,
  &nuvation_BMinModTemp,
  &nuvation_BTotDCCurr,
};


const int NUM_MOD_FAST_CHANNELS = 58;  // NUM_MOD_FAST_CHANNELS must = # channels in the array below
ModbusReg* mod_fast_regs[] = {
  &mppt600_vb_ref,
  &mppt600_adc_vb_f_med,
  &mppt600_adc_va_f_shadow,
  &mppt600_adc_ib_f_shadow,
  &mppt600_adc_ia_f_shadow,
  &mppt600_T_hs,
  &mppt600_mb_charge_state,
  &mppt600_fault_i,
  &mppt600_alarm_i,
  //&mppt600_Ahc_r,
  //&mppt600_kwhc_r,
  &mppt600_Whc_daily,

  &mppt60_vb_ref,
  &mppt60_adc_vb_f_med,
  &mppt60_adc_va,
  &mppt60_adc_ib_f_shadow,
  &mppt60_adc_ia_f_shadow,
  &mppt60_T_hs,
  &mppt60_charge_state,
  &mppt60_fault,
  &mppt60_alarm,
  //&mppt60_Ahc_r,
  &mppt60_whc_daily,
  &mppt60_EV_hvd,  // from EEPROM
  &mppt60_EV_hvr,  // from EEPROM

  &div60_V_ref,
  &div60_adc_vb_f,
  &div60_adc_vx_f,
  &div60_adc_ipv_f,
  &div60_adc_iload_f,
  &div60_T_hs,
  &div60_control_state,
  &div60_d_filt,
  &div60_fault,
  &div60_Alarm_LO,
  &div60_Alarm_HI,
  &div60_Ah_r,
  
  &mppt30_vb_ref,
  &mppt30_adc_vb_f_med,
  &mppt30_adc_va,
  &mppt30_adc_ib_f_shadow,
  &mppt30_adc_ia_f_shadow,
  &mppt30_T_hs,
  &mppt30_charge_state,
  &mppt30_fault,
  &mppt30_alarm,
  //&mppt30_Ahc_r,
  &mppt30_whc_daily,
  &mppt30_EV_hvd,  // from EEPROM
  &mppt30_EV_hvr,  // from EEPROM

  &div2_V_ref,
  &div2_adc_vb_f,
  &div2_adc_vx_f,
  &div2_adc_ipv_f,
  &div2_adc_iload_f,
  &div2_T_hs,
  &div2_control_state,
  &div2_d_filt,
  &div2_fault,
  &div2_Alarm_LO,
  &div2_Alarm_HI,
  &div2_Ah_r,
};


// Finally, expand the list below with the slow read (typ. EEPROM) channels...
// Array specifying which channels are to be read infrequently and sent out in UDP packets
const int NUM_MOD_SLOW_CHANNELS = 36;  // NUM_MOD_SLOW_CHANNELS value must match # channels in the array below
ModbusReg* mod_slow_regs[] = {
  &mppt60_EV_hvd,
  &mppt60_EV_hvr,
  &mppt30_EV_hvd,
  &mppt30_EV_hvr,
  //
  &mppt600_PV_P_0,
  &mppt600_PV_P_1,
  &mppt600_PV_P_2,
  &mppt600_PV_P_3,
  &mppt600_PV_P_4,
  &mppt600_PV_P_5,
  &mppt600_PV_P_6,
  &mppt600_PV_P_7,
  &mppt600_PV_P_8,
  &mppt600_PV_P_9,
  &mppt600_PV_P_10,
  &mppt600_PV_P_11,
  &mppt600_PV_P_12,
  &mppt600_PV_P_13,
  &mppt600_PV_P_14,
  &mppt600_PV_P_15,
  &mppt600_PV_V_0,
  &mppt600_PV_V_1,
  &mppt600_PV_V_2,
  &mppt600_PV_V_3,
  &mppt600_PV_V_4,
  &mppt600_PV_V_5,
  &mppt600_PV_V_6,
  &mppt600_PV_V_7,
  &mppt600_PV_V_8,
  &mppt600_PV_V_9,
  &mppt600_PV_V_10,
  &mppt600_PV_V_11,
  &mppt600_PV_V_12,
  &mppt600_PV_V_13,
  &mppt600_PV_V_14,
  &mppt600_PV_V_15,
};


// This function is called by getChanName(), which is called by printPOSTBody(), both in webclient.ino,
// and is used to select which subset of data channels gets put into UDP packets.
char* getModchannelName(int modbus_type, int i) {
  switch (modbus_type) {
    case 1:
      return(mod_fast_regs[i]->getChanLabel());
      break;
    case 2:
      return(mod_slow_regs[i]->getChanLabel());
      break;
    case 3:
      return(mod_nuv_regs[i]->getChanLabel());
      break;
    default:
      break;
  }
}

// This function is called by printPOSTBody() in webclient.ino
char* getModchannelValue(int modbus_type, int i) {
  switch (modbus_type) {
    case 1:
      return(mod_fast_regs[i]->valStrg());
      break;
    case 2:
      return(mod_slow_regs[i]->valStrg());
      break;
    case 3:
      return(mod_nuv_regs[i]->valStrg());
      break;
    default:
      break;
  }
}

#endif
