#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"
#include "uart.h"
#include <string>




void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial.println("Start");
  

}

void loop() {
  if(Serial1.available()){
    Serial.print(Serial1.readString());
    //string packet = Serial1.readString();
    //int packetLength = packet.length();
    //for(int i = 0; i < packetLength; i++){
      //if (packet[i] == ',') {
        //Serial.print()
      //}
    //}
    Serial.println();
  }
}
