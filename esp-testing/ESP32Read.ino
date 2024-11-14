 // Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17

#define RRC3_BAUD 9600

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial rrc3Serial(2);

void setup(){
  // Serial Monitor
  Serial.begin(9600);
  
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  rrc3Serial.begin(RRC3_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started at 9600 baud rate");
}

void loop(){
  Serial.println(" ");
  while (rrc3Serial.available() > 0){
    // get the byte data from the GPS
    char rrc3Data = rrc3Serial.read();
    Serial.print(rrc3Data);
  }
  
  
 
}