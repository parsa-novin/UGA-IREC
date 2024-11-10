from machine import UART,Pin

uart1 = UART(0,baudrate=9600, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)

while True:
    if uart1.any():
          rcvChar = uart1.read()
          line = rcvChar.decode("ascii") 
          print(line)
          
