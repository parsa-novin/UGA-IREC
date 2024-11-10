#include <stdio.h>
#include <stdlib.h>
#include <PicoSoftwareSerial.h>
#include "gpio.h"
#include "uart.h"
SoftwareSerial mySerial2(12, 11);
void setup() {
  Serial.begin(9600);
  Serial.println("Start");
  Serial1.begin(9600);
  mySerial2.begin(9600);
}

void loop() {
  int data = mySerial2.read();
  while (data != -1) {
    Serial.print((char) data);
    delay(3);
    data = mySerial2.read();
    if (data == -1) {
      Serial.println("");
    }
  }
}