#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"
#include "uart.h"

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial.println("Start");

}

void loop() {
  int data = Serial1.read();
  while(data != -1){
    Serial.println((char) data);
    data = Serial1.read();
  }
    

}
