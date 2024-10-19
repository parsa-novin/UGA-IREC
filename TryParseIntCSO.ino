#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"
#include "uart.h"
#include <string>

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial1.setTimeout(500); //Timeout after 1/2 second 

}

void loop() {
  int data = Serial1.read();
  if(Serial1.available()){
    data = Serial1.parseInt();
    Serial.println(data);
  }

}
