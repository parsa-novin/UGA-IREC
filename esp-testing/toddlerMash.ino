#include <Arduino.h>
#include <Preferences.h>

#define RXD2 16
#define TXD2 17

#define RRC3_BAUD 9600

#define PACKET_SIZE 32
#define NUM_PACKETS 16

HardwareSerial rrc3Serial(2);
Preferences preferences;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  rrc3Serial.flush();
  pinMode(RXD2, GPIO_PULLDOWN_ENABLE);

  rrc3Serial.begin(RRC3_BAUD, SERIAL_8N1, RXD2, TXD2, false, 100);
  Serial.println("Serial 2 started at 9600 baud rate");

  preferences.begin("myApp", false);
}

void loop() {
  static int packetCount = 0;
  static unsigned long lastPacketTime = 0;
  const unsigned long packetInterval = 1000; // 1 second interval

  if (millis() - lastPacketTime >= packetInterval) {
    lastPacketTime = millis();
    
    String gpsData = "";
    while (rrc3Serial.available()) {
      char incomingByte = rrc3Serial.read();
      if ((int)incomingByte == 13) {
        break; // End of packet
      }
      gpsData += incomingByte;
    }

    if (!gpsData.isEmpty()) {
      Serial.println("Received: " + gpsData);

      // Write packet to flash memory
      char key[10];
      snprintf(key, sizeof(key), "packet_%d", packetCount % NUM_PACKETS);
      preferences.putBytes(key, gpsData.c_str(), min(PACKET_SIZE, static_cast<int>(gpsData.length())));
      Serial.printf("Wrote packet %d\n", packetCount % NUM_PACKETS);
      packetCount++;

      // Read and print the last stored packet
      uint8_t packet[PACKET_SIZE];
      size_t readBytes = preferences.getBytes(key, packet, PACKET_SIZE);

      if (readBytes > 0) {
        Serial.printf("Read packet %d: ", packetCount % NUM_PACKETS);
        for (int j = 0; j < readBytes; j++) {
          Serial.printf("%02X ", packet[j]);
        }
        Serial.println();
      } else {
        Serial.printf("Failed to read packet %d\n", packetCount % NUM_PACKETS);
      }
    }
  }
}
