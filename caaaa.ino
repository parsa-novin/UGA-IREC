#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"
#include "uart.h"
#include <string>




void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  

}

void loop() {
  int data = Serial1.read();
  int aaa = 1;
  while (data != -1) {
    Serial.print((char) data);
    data = Serial1.read();
    aaa = 0;
  }
  if (aaa = 0) {
    Serial.println("");
    aaa = 1;
  }
  /*if(Serial1.available()){
    Serial.print(Serial1.readString());
    //string packet = Serial1.readString();
    //int packetLength = packet.length();
    //for(int i = 0; i < packetLength; i++){
      //if (packet[i] == ',') {
        //Serial.print()
      //}
    //}
    Serial.println();
  }*/
}
