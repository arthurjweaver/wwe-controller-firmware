// ---------- temperature.h ----------
//

// We're going to assume that there's a single DS18B20 attached to the pin.

class DS18B20: public OneWire {
  public:
    DS18B20(int pin): OneWire(pin){
    }

    // get the addr of the first chip found with a search();
    int init(int pin) {
      present = search(addr);
      // the first ROM byte indicates which chip
      switch (addr[0]) {
        case 0x10:
          //Serial.println("temperature.h: Chip = DS18S20");  // or old DS1820
          type_s = 1;
          break;
        case 0x28:
          //Serial.println("temperature.h: Chip = DS18B20");
          type_s = 0;
          break;
        case 0x22:
          //Serial.println("temperature.h: Chip = DS1822");
          type_s = 0;
          break;
        default:
          Serial.println("temperature.h: Device is NOT a DS18x20 family device.");
      } 
    }

    // We need to call this and wait about a second for the chip to be done with the 
    void convert() {
      OneWire::reset();
      OneWire::select(addr);
      OneWire::write(0x44, 1);
      // this needs to be used in such a way that there is a 1-sec delay between this and 
      // reading the temperature.
    }

    float readTemp() {
      OneWire::reset();
      OneWire::select(addr);
      OneWire::write(0xBE, 1);     
      for(i = 0; i < 9; i++){
        data[i] = OneWire::read();
      }
      //Serial.print("temperature: CRC = ");
      //Serial.print(OneWire::crc8(data, 8), HEX);
      //Serial.println();

      // Convert the data to actual temperature
      // because the result is a 16 bit signed integer, it should
      // be stored to an "int16_t" type, which is always 16 bits
      // even when compiled on a 32 bit processor.
      int16_t raw = (data[1] << 8) | data[0];
      if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
          // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
      }
      celsius = (float)raw / 16.0;
      fahrenheit = celsius * 1.8 + 32.0;
      //Serial.print("temperature: controller T = ");
      //Serial.print(celsius);
      //Serial.print(" C, ");
      //Serial.print(fahrenheit);
      //Serial.println(" F");
      return(celsius);
    }

  private:
    int type_s = 0;
    byte i; 
    // DS18B20 has been located
    int present;
    byte data[12];
    byte addr[8];
    float celsius = 0.0;
    float fahrenheit = 0.0;
};
