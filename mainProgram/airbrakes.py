import time

def testFunction(az, t):
    global airbrakesServo
    if(az < 0 and t > 0):
        airbrakesServo+=2
        return (500 + (2000 * airbrakesServo) // 180)
    else:
        return 1333
def waitAndReset(delay):
    time.sleep(delay)
    global airbrakesServo
    airbrakesServo = 75
