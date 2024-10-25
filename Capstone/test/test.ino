#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"
#include "uart.h"

void setup() {
  Serial.begin(9600);
  Serial.println("Start");

}

void loop() {
  char in = Serial.read();
  while(in == '1'){
    digitalWrite(22, HIGH);
    delay(500);
  }
  int response = analogRead(26);
  Serial.println("Electrode: " + response);
  }
