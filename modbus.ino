// ---------- modbus.ino ----------
// Modbus initialization

void enableRS485() {
  digitalWrite(RS485_ENBL_PIN, 1);
}
void disableRS485() {
  digitalWrite(RS485_ENBL_PIN, 0);
  //delay(10);
}

void initModbus() {
  Serial << "modbus: Initializing Modbus...\n";
  Serial3.begin(9600);  // Modbus/RTU (RS485) uses the Serial3 interface on the SAM DUE board
  //
  // Start ModbusMasters for each Modbus device and set callback functions for before and after.
  //
  // Morningstar TS-MPPT-600V (WIND)
  mppt600.begin(1, Serial3);
  mppt600.preTransmission(enableRS485);
  mppt600.postTransmission(disableRS485);

  // Morningstar TS-60 (DIV1)
  div60.begin(2, Serial3);
  div60.preTransmission(enableRS485);
  div60.postTransmission(disableRS485);

  // Morningstar TS-MPPT-60 (PV2)
  mppt60.begin(3, Serial3);
  mppt60.preTransmission(enableRS485);
  mppt60.postTransmission(disableRS485);
  //mppt60.setUnitId(3);  // uncomment if TCP instead of RTU

  // Morningstar TS-MPPT-30 (PV1)
  mppt30.begin(4, Serial3);
  mppt30.preTransmission(enableRS485);
  mppt30.postTransmission(disableRS485);

  // Morningstar TS-60 (DIV2)
  div2.begin(5, Serial3);
  div2.preTransmission(enableRS485);
  div2.postTransmission(disableRS485);


  // Set # of CONTIGUOUS regs for Morningstar controller long reads.
  // IMPORTANT: Set these vals accurately because it affects total Modbus read time!
  // Refer to Channel Definitions in modbus.h and find the lowest and highest register # accessed by the long read. 
  // The difference (+1) is the arg to setNumRegs() below.
  mppt600_cache.setNumRegs(45);
  mppt60_cache.setNumRegs(45);
  mppt30_cache.setNumRegs(45);
  div60_cache.setNumRegs(22);
  div2_cache.setNumRegs(22);

  // Do Modbus long reads and check how much time each requires. But also...
  // MUST do these long reads here in order to set the various <device>_cache_addr vars in readReg() - see modbus.h
  // The total time of these long reads (+ subsequent UDP broadcasts) MUST be less 
  // than 1 SECOND if we're going to collect data at that rate!
  // The delays after readReg(false) are REQUIRED to avoid Response Timeout errors - see modbus.h
  int total_time = millis();

  int cache_time = millis(); mppt600_cache.readReg(false); delay(5); // false --> do a direct read
  Serial << "modbus: Modbus mppt600_cache long read = " << (millis() - cache_time) << " msec\n";
  cache_time = millis(); div60_cache.readReg(false); delay(5);
  Serial << "modbus: Modbus div60_cache long read = " << (millis() - cache_time) << " msec\n";
  cache_time = millis(); mppt30_cache.readReg(false); delay(5);
  Serial << "modbus: Modbus mppt30_cache long read = " << (millis() - cache_time) << " msec\n";
  cache_time = millis(); mppt60_cache.readReg(false); delay(5);
  Serial << "modbus: Modbus mppt60_cache long read = " << (millis() - cache_time) << " msec\n";
  cache_time = millis(); div2_cache.readReg(false); delay(5);
  Serial << "modbus: Modbus div2_cache long read = " << (millis() - cache_time) << " msec\n";

  Serial << "modbus: Total Modbus long read time = " << (millis() - total_time) << " msec\n";
  
  // Read Morningstar controller charge_state regs here BEFORE readADCs() calls manageDumpLoad() for the first time.
  // This is needed to determine whether the controllers are DISConnected or not - see furlctl.ino
  mppt600_mb_charge_state.readReg(false); delay(5);
  mppt30_charge_state.readReg(false); delay(5);
  mppt60_charge_state.readReg(false); delay(5);

  // Read Morningstar controller serial #, firmware version, hardware version.
  // Morningstar serial# is ascii chars: byte --> ascii --> string
  // Morningstar firmware version is BCD: select high byte, multiply it by 10, add low byte
  // Morningstar hardware version is highbyte.lowbyte
  // NOTES: There is no software version register for the TS-60.
  //        The delay(5) after each readReg(false) is REQUIRED to avoid a "Response Timeout" error - see modbus.h
  //        One could experiment with shorter delays.
  // TS-MPPT-600V
  mppt600_Eserial0.readReg(false); delay(5);
  mppt600_Eserial1.readReg(false); delay(5);
  mppt600_Eserial2.readReg(false); delay(5);
  mppt600_Eserial3.readReg(false); delay(5);
  mppt600_ver_sw.readReg(false); delay(5);
  mppt600_Ehw_version.readReg(false); delay(5);
  String mppt600_sn = String(char(lowByte(mppt600_Eserial0.valInt()))) + String(char(highByte(mppt600_Eserial0.valInt()))) +
                      String(char(lowByte(mppt600_Eserial1.valInt()))) + String(char(highByte(mppt600_Eserial1.valInt()))) +
                      String(char(lowByte(mppt600_Eserial2.valInt()))) + String(char(highByte(mppt600_Eserial2.valInt()))) +
                      String(char(lowByte(mppt600_Eserial3.valInt()))) + String(char(highByte(mppt600_Eserial3.valInt())));
  int mppt600_sw = ((mppt600_ver_sw.valInt() & 0xf0) >> 4)*10 + (mppt600_ver_sw.valInt() & 0x0f);
  String mppt600_hw = String(highByte(mppt600_Ehw_version.valInt())) + String(".") + String(lowByte(mppt600_Ehw_version.valInt()));
  Serial << "modbus: mppt600 serial number = " << mppt600_sn << "\n";
  Serial << "modbus: mppt600 firmware version = " << mppt600_sw << "\n";
  Serial << "modbus: mppt600 hardware version = " << mppt600_hw << "\n";

  // TS-MPPT-60
  mppt60_Eserial0.readReg(false); delay(5);
  mppt60_Eserial1.readReg(false); delay(5);
  mppt60_Eserial2.readReg(false); delay(5);
  mppt60_Eserial3.readReg(false); delay(5);
  mppt60_ver_sw.readReg(false); delay(5);
  mppt60_Ehw_version.readReg(false); delay(5);
  String mppt60_sn = String(char(lowByte(mppt60_Eserial0.valInt()))) + String(char(highByte(mppt60_Eserial0.valInt()))) +
                     String(char(lowByte(mppt60_Eserial1.valInt()))) + String(char(highByte(mppt60_Eserial1.valInt()))) +
                     String(char(lowByte(mppt60_Eserial2.valInt()))) + String(char(highByte(mppt60_Eserial2.valInt()))) +
                     String(char(lowByte(mppt60_Eserial3.valInt()))) + String(char(highByte(mppt60_Eserial3.valInt())));
  int mppt60_sw = ((mppt60_ver_sw.valInt() & 0xf0) >> 4)*10 + (mppt60_ver_sw.valInt() & 0x0f);
  String mppt60_hw = String(highByte(mppt60_Ehw_version.valInt())) + String(".") + String(lowByte(mppt60_Ehw_version.valInt()));
  Serial << "modbus: mppt60 serial number = " << mppt60_sn << "\n";
  Serial << "modbus: mppt60 firmware version = " << mppt60_sw << "\n";
  Serial << "modbus: mppt60 hardware version = " << mppt60_hw << "\n";

  // TS-MPPT-30
  mppt30_Eserial0.readReg(false); delay(5);
  mppt30_Eserial1.readReg(false); delay(5);
  mppt30_Eserial2.readReg(false); delay(5);
  mppt30_Eserial3.readReg(false); delay(5);
  mppt30_ver_sw.readReg(false); delay(5);
  mppt30_Ehw_version.readReg(false); delay(5);
  String mppt30_sn = String(char(lowByte(mppt30_Eserial0.valInt()))) + String(char(highByte(mppt30_Eserial0.valInt()))) +
                     String(char(lowByte(mppt30_Eserial1.valInt()))) + String(char(highByte(mppt30_Eserial1.valInt()))) +
                     String(char(lowByte(mppt30_Eserial2.valInt()))) + String(char(highByte(mppt30_Eserial2.valInt()))) +
                     String(char(lowByte(mppt30_Eserial3.valInt()))) + String(char(highByte(mppt30_Eserial3.valInt())));
  int mppt30_sw = ((mppt30_ver_sw.valInt() & 0xf0) >> 4)*10 + (mppt30_ver_sw.valInt() & 0x0f);
  String mppt30_hw = String(highByte(mppt30_Ehw_version.valInt())) + String(".") + String(lowByte(mppt30_Ehw_version.valInt()));
  Serial << "modbus: mppt30 serial number = " << mppt30_sn << "\n";
  Serial << "modbus: mppt30 firmware version = " << mppt30_sw << "\n";
  Serial << "modbus: mppt30 hardware version = " << mppt30_hw << "\n";

  
  // TS-MPPT-60: read and set scale factors
  mppt60_V_PU.readReg(false); delay(5);
  float mppt60_V_scale = mppt60_V_PU.valInt() / 65536.0;
  Serial << "modbus: mppt60_V_scale = " << mppt60_V_scale << "\n";
  mppt60.setScaleV(mppt60_V_scale);
  //mppt60.setScaleV(180.0);  // explicitly set scale factor

  mppt60_I_PU.readReg(false); delay(5);
  float mppt60_I_scale = mppt60_I_PU.valInt() / 65536.0;
  Serial << "modbus: mppt60_I_scale = " << mppt60_I_scale << "\n";
  mppt60.setScaleI(mppt60_I_scale);
  //mppt60.setScaleI(80.0);

  // This scale factor is set here rather than making a special case for it in modbus.h
  mppt60_Ahc_r.setScale(0.1);

  // TS-MPPT-30: read and set scale factors
  mppt30_V_PU.readReg(false); delay(5);
  float mppt30_V_scale = mppt30_V_PU.valInt() / 65536.0;
  Serial << "modbus: mppt30_V_scale = " << mppt30_V_scale << "\n";
  mppt30.setScaleV(mppt30_V_scale);
  //mppt30.setScaleV(180.0);  // comment out if scale factor from cache is accurate

  mppt30_I_PU.readReg(false); delay(5);
  float mppt30_I_scale = mppt30_I_PU.valInt() / 65536.0;
  Serial << "modbus: mppt30_I_scale = " << mppt30_I_scale << "\n";
  mppt30.setScaleI(mppt30_I_scale);
  //mppt30.setScaleI(80.0);  // comment out if scale factor from cache is accurate

  // This scale factor is set here rather than making a special case for it in modbus.h
  mppt30_Ahc_r.setScale(0.1);

  // TS-60 (DIV1): Set fixed scale factors
  div60_V_ref.setScale(96.667 / 32768.0);
  div60_adc_vb_f.setScale(96.667 / 32768.0);
  div60_adc_iload_f.setScale(316.67 / 32768.0);
  div60_d_filt.setScale(100.0 / 230.0);
  div60_Ah_r.setScale(0.1);
  
  // TS-60 (DIV2): Set fixed scale factors
  div2_V_ref.setScale(96.667 / 32768.0);
  div2_adc_vb_f.setScale(96.667 / 32768.0);
  div2_adc_iload_f.setScale(316.67 / 32768.0);
  div2_d_filt.setScale(100.0 / 230.0);
  div2_Ah_r.setScale(0.1);

  // Now that we've finished long reads of 'fast' RAM data into the response buffer of each device...
  // we're free to access the modbus again, this time getting 'slow' EEPROM data from each device,
  // one parm at a time. Long reads of EEPROM are not necessary because we don't need to save time!
  // IMPORTANT: We have to do this LAST because doing a readReg(false) OVERWRITES the 'fast' data 
  // in the response buffers!

  // Read Morningstar EEPROM regs, one at a time. 
  // This is OK because these channels are rarely changed and are sent out on UDP 'infrequently'.
  // As with the 'fast' channel long reads, the delay is REQUIRED.
  total_time = millis();
  for (int i = 0; i < NUM_MOD_SLOW_CHANNELS; i++) {
    mod_slow_regs[i]->readReg(false);  // false = read register(s) directly
    delay(5);
  }
  Serial << "modbus: Modbus 'slow' register read time = " << (millis() - total_time) << " msec\n";

  // Print each slow channel name and value
  for (int i = 0; i < NUM_MOD_SLOW_CHANNELS; i++) {
    Serial << "modbus: " << "channel name = " << mod_slow_regs[i]->getChanName() 
                         << ", channel value = " << mod_slow_regs[i]->valStrg() << "\n";
  }

  // Get Nuvation Modbus/TCP data --> ***requires Ethernet***
  //   For Nuvation parm defs, see Energy-Storage-Information-Models_D3-2015-10-26-Update.xlsx spreadsheet.
  //   Scale factors are represented as powers of 10, e.g., -3 means divide by 1000.
  if ( ethernetOK() ) {
    Serial << "wwe: Reading and setting Nuvation scale factors...\n";
    nuvation_Vol_SF.readReg(false);
    nuvation_MaxBatA_SF.readReg(false);
    nuvation_BCellVol_SF.readReg(false);
    nuvation_BModTemp_SF.readReg(false);
    nuvation_BCurrent_SF.readReg(false);
    //Serial << "wwe: " << "nuvation_Vol_SF = " << nuvation_Vol_SF.valInt() << "\n";  // scale factors are MOD_HALFWORD_SIGNED = unsigned 16-bit int
    //Serial << "wwe: " << "nuvation_MaxBatA_SF = " << nuvation_MaxBatA_SF.valInt() << "\n";
    //Serial << "wwe: " << "nuvation_BCellVol_SF = " << nuvation_BCellVol_SF.valInt() << "\n";
    //Serial << "wwe: " << "nuvation_BModTemp_SF = " << nuvation_BModTemp_SF.valInt() << "\n";
    //Serial << "wwe: " << "nuvation_BCurrent_SF = " << nuvation_BCurrent_SF.valInt() << "\n";
    float sf = pow(10, nuvation_Vol_SF.valInt());  // apply scale factors
    nuvation_Vol.setScale(sf);
    sf = pow(10, nuvation_MaxBatA_SF.valInt());
    nuvation_MaxBatACha.setScale(sf);
    nuvation_MaxBatADischa.setScale(sf);
    sf = pow(10, nuvation_BCellVol_SF.valInt());
    nuvation_BMaxCellVol.setScale(sf);
    nuvation_BMinCellVol.setScale(sf);
    sf = pow(10, nuvation_BModTemp_SF.valInt());
    nuvation_BMaxModTemp.setScale(sf);
    nuvation_BMinModTemp.setScale(sf);
    sf = pow(10, nuvation_BCurrent_SF.valInt());
    nuvation_BTotDCCurr.setScale(sf);
  }
}
