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
  if(Serial1.available()){
    data = Serial1.read();
    Serial.println(data);
  }
}
