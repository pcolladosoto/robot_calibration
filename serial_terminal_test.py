import serial
import sys

port = serial.Serial("/dev/rfcomm1", baudrate = 38400, timeout = 0)

while True:
    if port.in_waiting > 0:
        char = port.read(1)
        print("%s" % (char.decode()), end="")
        sys.stdout.flush()
        port.flush()
