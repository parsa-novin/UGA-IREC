from machine import UART, Pin
import time
uart0 = UART(0,baudrate=9600, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)
uart1 = UART(1,baudrate=9600, tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1)

i: int = 0
print(i)
read = input("put what u want here: ")
while read == "a":
     print(i)
     print(uart0.read(1))
     print(i)
     #uart2.write("Packet #" + i)
     #uart2.write(uart1.read())
     i+=1
     read = input("here: ")
     
        #  f = open("demo.txt", "a")  rcvChar.decode("ascii"),end=""
        # f.write(line)