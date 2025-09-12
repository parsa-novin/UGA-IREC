#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include "gpio.h"
#include "uart.h"
using namespace std;

struct DataStruct{
  float timestamp;
  int altitude;
  int velocity;
  int temp;
  float voltage;
};
vector<DataStruct> data;

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial.println("Start");
}

void loop() {
  if (Serial.available()) {
    string dataString = Serial.readString();
    storeData(dataString);
    Serial.println(data.back().temp);
  }
}

void storeData(string dataString) {
  int index = 1;
  DataStruct dataStruct;
  for (int i = 1; i <= 5; i++) {
    if (dataString[index] == ',') {
      switch(i) {
      case 1: 
        dataStruct.timestamp = (float) dataString.substr(0, index);
        dataString = dataString.substr(index, dataString.length() - index);
        break;       
      case 2:
        dataStruct.altitude = (int) dataString.substr(0, index);
        dataString = dataString.substr(index, dataString.length() - index);
        break;
      case 3:
        dataStruct.velocity = (int) dataString.substr(0, index);
        dataString = dataString.substr(index, dataString.length() - index);
        break;
      case 4:
        dataStruct.temp = (int) dataString.substr(0, index);
        dataString = dataString.substr(index, dataString.length() - index);
        if(dataString.find(',')) {
          dataString[3] = ',';
        }
        break;
      case 5:
        dataStruct.voltage = (float) dataString.substr(0, index);
        dataString = dataString.substr(index, dataString.length() - index);
        data.push_back(dataStruct);

        // Recursive call if dataString has too many items
        if (dataString.length() != 0) {
          storeData(dataString);
        }
        return;
      default: {
        Serial.println("There has been an error in the switch case");
        break;
      }
      index = 0;
    }
    index++;
  }
}
