import struct
import serial

start_bye = '0xAA'
movement_packet = 1
rrc3_packet = 2

header_format = '<BBB' #3 unsigned chars
movement_format = '<ffffff' #6 floats
rrc3_format = '<fiih3s' #1 float, 2 signed ints (32 bits), 1 signed int (16 bits), 3 bytes of string data

def readMessage():
    header_serial = serial.read(3) #len(header_ser) = 3
    startByte, packetType, payloadLength = struct.unpack(header_format, header_serial) #struct.unpack('<BBB', [raw data stream]) -> little endian, 3 unsigned chars

# movement and rrc3 are tuples/arrays by default but you can store them into individual variables slike how the header is above
# you can verify packet integrity with matching startbyte, 1 or 2 pakcet type, and matching payload length


while True:
    message = readMessage()