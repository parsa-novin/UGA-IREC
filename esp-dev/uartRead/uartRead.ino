 // Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17

#define RRC3_BAUD 9600

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial rrc3Serial(2);

void setup(){
  pinMode(RXD2, GPIO_PULLDOWN_ENABLE); 
  // Serial Monitor
  Serial.begin(RRC3_BAUD);
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  rrc3Serial.begin(RRC3_BAUD, SERIAL_8N1, RXD2, TXD2, false, 100);
  Serial.print("Serial 2 started at 9600 baud rate");
}


void loop(){
  static int count = 0;
  static String gpsData = "";  // Buffer to accumulate incoming data
  while (rrc3Serial.available()) {
    char incomingByte = rrc3Serial.read();  // Read a byte
    if ((int) incomingByte == 13 || count > 16) {  // End of a packet (NMEA sentence)
      Serial.println(gpsData);  // Process the complete GPS data
      gpsData = "";  // Reset buffer for next packet
      count = 0;
      if((int) incomingByte == 13){
      Serial.println("carriage return");
      }
    } else {
      gpsData += incomingByte;  // Add the byte to the buffer
      count ++;
    }
  }
}