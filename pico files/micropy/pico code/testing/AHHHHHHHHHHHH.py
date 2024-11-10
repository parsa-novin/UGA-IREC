from machine import UART,Pin

uart1 = UART(0,baudrate=9600, tx=Pin(12), rx=Pin(13), bits=8, parity=None, stop=1)
uart2 = UART(0,baudrate=9600, tx=Pin(8), rx=Pin(9), bits=8, parity=None, stop=1)

while True:
    #if uart1.any():
          #rcvChar = uart1.read(1)
          #line = rcvChar.decode("ascii")
          uart2.write(200)
          
        #  f = open("demo.txt", "a")  rcvChar.decode("ascii"),end=""
        # f.write(line)
