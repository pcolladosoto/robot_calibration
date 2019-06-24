import serial
from re import findall
from time import sleep as ts

ard = serial.Serial("/dev/ttyUSB0", baudrate = 38400, timeout = None)

ts(5)

ard.write("41000".encode())

ard.reset_input_buffer()

str = ard.readline().decode()

print(str)

finished = 0

while True:
    if ard.read().decode() == '.':
        print("Got a dot!\n", end="")
        finished += 1
        if finished == 2:
            break

ard.close()

exit()
