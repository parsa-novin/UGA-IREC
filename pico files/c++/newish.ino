#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "gpio.h"
#include "uart.h"
using namespace std;

int dataArray[50];
int i = 0;

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial.println("Start");
}

void loop() {
  Serial1.println(Serial1.read());
    i++;
  }
  i = 0;
  Serial1.println("bb");
  for (int j : dataArray) {
    Serial1.println("aa");
  }
}