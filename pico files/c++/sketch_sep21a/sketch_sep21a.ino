#include <stdio.h>
#include <stdlib.h>
#include "gpio.h"
#include "uart.h"

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);

}

void loop() {
  Serial.print(Serial1.available());
  if(Serial1.available())
    Serial.println(Serial1.read());

}
