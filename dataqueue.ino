// ---------- dataqueue.ino ----------
// Implement a data queue using SD card. This is an ordinary file to which we append JSON strings terminated by '\n'. 
// The SD card is huge, and we just keep appending to it until it grows to some large size (xx bytes), then we
// erase the file and start over.
//
// Data format:
// {"time": <unixtime>, "vals": [<val1>, <val2>, <val3>]}

// CURRENTLY UNUSED (2022-08-18)

/*

#include <JsonGenerator.h>   // ~/Documents/Arduino/libraries/ArduinoJson

//using namespace ArduinoJson::Generator;
//JsonObject<6> genroot;
//JsonArray<10> data_array;

// Start reading here.
long read_pointer = 0;
long write_pointer = 0;

void initDataQueue(){
  int i;
  for(i = 0; i < 14; i++){
    //data_array.add<2>(0.0);
  }
}

char* putDataLine(){
  int i;
  for(i = 0; i < 14; i++){
    //data_array.items[i] = getChannelRMS(i);
  }  
}

char* getNextDataLine(){  
}

*/
