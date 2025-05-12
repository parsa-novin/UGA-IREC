import time
import types
import board
import busio
import inspect
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
)


i2c  = busio.I2C(board.D1, board.D0)
imu  = BNO08X_I2C(i2c, address=0x4A, debug=False)
imu.soft_reset()
# show the source of the library?s handler
print(inspect.getsource(imu._handle_packet))
# show the file where it?s defined
print(imu._handle_packet.__code__.co_filename)
imu._sequence_number = [0] * 1000
orig = imu._process_available_packets
imu._process_available_packets = lambda max_packets=None: orig(max_packets=1)
time.sleep(0.1)
imu.initialize()
time.sleep(0.1)

imu.enable_feature(BNO_REPORT_ACCELEROMETER)
time.sleep(0.1)
imu.enable_feature(BNO_REPORT_GYROSCOPE)
time.sleep(0.1)
imu.enable_feature(BNO_REPORT_MAGNETOMETER)
time.sleep(0.1)


def getVectors():
    while True:
        #try:
        print("getVectors called!")
        a = imu.acceleration
        g = imu.gyro
        m = imu.magnetic
#         except (RuntimeError,OSError):
#             if OSError:
#                 print("loser")
#             elif RuntimeError:
#                 print("tanay doo doo head")
#             continue          # skip if the driver was busy parsing
        vals = (float(a[0]), float(a[1]), float(a[2]), float(g[0]), float(g[1]), float(g[2]), float(m[0]), float(m[1]), float(m[2]))
        try:
            return(vals)
        except (serial.SerialTimeoutException, OSError):
            pass              # UART TX buffer full ? drop this sample
def getRRC3(s):
    parts = s.split(',')
    if len(parts) != 5:
        return None
    ts, alt, vel, temp, event = parts
    try:
        return {
            'timestamp':   float(ts),
            'altitude':    int(alt),
            'velocity':    int(vel),
            'temperature': int(temp),
            'event':       event[:3]
        }
    except ValueError:
        return None
