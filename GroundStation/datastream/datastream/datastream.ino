#include <M5Unified.h>

#define PACKET_TYPE_MOVEMENT 1
#define PACKET_TYPE_RRC3DATA 2
#define START_BYTE 0xAA

#pragma pack(1)
typedef struct {
  uint8_t startByte;
  uint8_t packetType;
  uint8_t payloadLength;
} PacketHeader;

#pragma pack(1) 
typedef struct{
  float accel[3];
  float gyro[3];
} Movement;
Movement m;

#pragma pack(1)
typedef struct{
  float timestamp;
  int32_t altitude;
  int32_t velocity;
  int16_t temperature;
  char event[3];
} rrc3Data;
rrc3Data record;

String generateDataStream() {
  // 1. Generate a random timestamp: a float from 0.0 to 9999.9 (one decimal)
  float timestamp = random(0, 100000) / 10.0;
  
  // 2. Generate AGL Altitude: a signed integer from -99999 to +99999
  int altitude = random(0, 199999) - 99999;
  
  // 3. Generate Velocity: a signed integer from -9999 to +9999
  int velocity = random(0, 19999) - 9999;
  
  // 4. Generate Temperature: a signed integer from -999 to +999
  int temperature = random(0, 1999) - 999;
  
  // 5. Choose an event string from a set of options.
  const char* eventOptions[] = {"???", "dma", "DMA"};
  int eventChoice = random(0, 3);

  // Build the final comma-separated string.
  // We include a plus sign for positive values.
  String output = "";
  output += String(timestamp, 1);  // timestamp with one decimal
  output += ",";
  output += String(altitude);
  output += ",";
  output += String(velocity);
  output += ",";
  output += String(temperature);
  output += ",";
  output += String(eventOptions[eventChoice]);
  
  return output;
}

void parseDataStream(const String &data) {
  int pos1 = data.indexOf(',');
  int pos2 = data.indexOf(',', pos1 + 1);
  int pos3 = data.indexOf(',', pos2 + 1);
  int pos4 = data.indexOf(',', pos3 + 1);
  int pos5 = data.indexOf(',', pos4 + 1);
  
  if (pos1 == -1 || pos2 == -1 || pos3 == -1 || pos4 == -1 || pos5 != -1) {
    return;
  }

  String tsStr = data.substring(0, pos1);
  String altStr = data.substring(pos1 + 1, pos2);
  String velStr = data.substring(pos2 + 1, pos3);
  String tempStr = data.substring(pos3 + 1, pos4);
  String eventStr = data.substring(pos4 + 1);

  record.timestamp = tsStr.toFloat();
  record.altitude = altStr.toInt();
  record.velocity = velStr.toInt();
  record.temperature = tempStr.toInt();
  eventStr.toCharArray(record.event, sizeof(record.event));
}

void sendPacket(uint8_t packetType, const uint8_t* payload, uint8_t payloadLength) {
  PacketHeader header;
  header.startByte = START_BYTE;
  header.packetType = packetType;
  header.payloadLength = payloadLength;

  // Transmit header and payload
  Serial.write((uint8_t*)&header, sizeof(header));
  Serial.write(payload, payloadLength);
}

void setup() {
  M5.begin();
  Serial.begin(115200);
}

void loop() {
  M5.Imu.getAccelData(&m.accel[0], &m.accel[1], &m.accel[2]);
  M5.Imu.getGyroData(&m.gyro[0], &m.gyro[1], &m.gyro[2]);
  
  sendPacket(PACKET_TYPE_MOVEMENT, (uint8_t*)&m, sizeof(m));

  parseDataStream(generateDataStream());

  sendPacket(PACKET_TYPE_RRC3DATA, (uint8_t*)&record, sizeof(record));
  delay(500);
}
