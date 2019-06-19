#Imports
import glob #For finding the serial terminal
import serial #For working witht the serial ports
import os #For system calls to ls and rm
from numpy import convolve #For the convolution
from re import findall #For parsing the input strings
from math import pi, log10 #We need our friend PI and to check the digits of the circumference!
import sys #To flush the print function when debugging!
import time #Debuggig!


#Constants
ARDUINO_BAUDRATE = 38400
REDUCING_FACTOR = 50.9 #The reducer makes axis speed = central_speed / 50.9.
ENC_PULSES_PER_REV = 3 #The encoders emit 3 pulses per rev
NOM_DIAMETER = 190 #In mm
WHEELBASE = 590 #In mm
N_TURNS = 8
ED = 1
ES = 1
K = -1 #Computed later on
PULSES_PER_REV = -1 #Computed later on
MM_TO_PULSES = -1 #Computed later on
ESTIMATED_PULSES_PER_TURN = -1 #Computed later on
REAL_PULSES_PER_TURN = -1 #Computed later on

#Errors
ERROR_MESSAGES = ["PORT NOT FOUND, ABORTING!\n",
                  "SIGNAL LENGTHS ARE DIFFERENT, ABORTING!\n",
                  "PORT NOT REACHABLE\n"]
PORT_NOT_FOUND = 0
ERROR_SIGNAL_TREATMENT = 1
PORT_NOT_REACHABLE = 2

#Commands
GET_DATA = 'F'
TURN_L_WHEEL_STOPPED = 'C0000371' #XXXX is the distance in cm. Default is 2 * PI * 590 mm
TURN_R_WHEEL_STOPPED = 'D0000371' #XXXX is the distance in cm. Default is 2 * PI * 590 mm
TURN_BOTH_WHEELS = '3360' #XXX is (I guess) the angle to turn in deg
GO_STRAIGHT_3M = '43000' #XXXX is the distance in mm
GO_STRAIGHT_1M = '41000' #Used for debugging!

def find_N_open_serial_port():
    #Expand all the paths in the argument. [A-Za-z] matches any letter both upper and lower case. Its a RegExpr
    #glob.glob("/dev/tty[A-Za-z]*")
    #Maybe this'll work too
    candidates = glob.glob('/dev/tty[A-Za-z]*')

    #open_ports = []
    #Try to open each ttyUSBX port. If it's not being used, throw the exception and continue
    for path in candidates:
        print("Checking: %s\n" % (path), end="")
        try:
            #port = serial.Serial(path, ARDUINO_BAUDRATE, timeout = None)
            port = serial.Serial(path, baudrate = ARDUINO_BAUDRATE, timeout = None)
            if check_if_arduino():
                print("Found Arduino at port %s\n" % (port.name), end="")
                port.close()
                return port
            else:
                port.close()
        except(OSError, serial.SerialException):
            print("%s" % (ERROR_MESSAGES[PORT_NOT_REACHABLE]), end="")
            pass

    #return open_ports
    return PORT_NOT_FOUND

def check_if_arduino():
    #Kind of hacky... Write the output of the given folder to tmp and read from it to see if it's an Arduino. If ls returns a non-zero value we have an error! Our Arduino is not original, so we have to look for another string...
    if os.system("ls /dev/serial/by-id > tmp") != 0 or not ('USB2.0-Serial-if00' in open('tmp', 'r').read()):
        os.system("rm tmp")
        return False
    os.system("rm tmp")
    return True

def get_data(ard):
    #dumped_data = open("dumped_data.txt", "w+")
    #dumped_data.write("Test!\n")
    #port.write(GET_DATA.encode())

    #while port.in_waiting > 0:
    #    dumped_data.write(port.readline().decode())

    #dumped_data.seek(0)

    #return dumped_data

    dumped_data = open("data.txt", "w+")

    ard.write("F".encode())

    newline = ""

    while newline != "END\r\n":
        newline = (ard.readline()).decode()
        dumped_data.write(newline)

    dumped_data.seek(0)
    dumped_data.close()

    return dumped_data

def print_file(data):
    data = open("data.txt", "r")

    for line in data:
        print(line, end = "")

    data.close()

    return

def execute_command(command, port):
    finished = 0
    port.open()

    if command == "Get data":
        print("Let's go get that data!\n", end="")
        return get_data(port)
    elif command == "Turn L stopped":
        print("Turning with the L wheel stopped!\n", end="")
        port.write(TURN_L_WHEEL_STOPPED.encode())
        while True:
            if port.read().decode() == '.':
                print("Got the first dot!\n", end="")
                finished += 1
                if finished == 2:
                    port.close()
                    break
    elif command == "Turn R stopped":
        print("Turning with the R wheel stopped!\n", end="")
        port.write(TURN_R_WHEEL_STOPPED.encode())
        while True:
            if port.read() == '.':
                finished += 1
                if finished == 2:
                    break
    elif command == "Turn both wheels":
        print("Turning with both wheels!\n", end="")
        port.write(TURN_BOTH_WHEELS.encode())
        while True:
            if port.read() == '.':
                finished += 1
                if finished == 2:
                    break
    elif command == "Straight 3 m":
        print("Going straight for 3 m!\n", end="")
        #port.write('41000'.encode())
        #port.flush()
        port.write(GO_STRAIGHT_1M.encode())
        while True:
            char = port.read().decode()
            print("%s" % (char), end="")
            sys.stdout.flush()
            if char == ".":
                print("Hey!\n", end="")
                finished += 1
                if finished == 2:
                    break

    return port.close()

def initial_data():

    global REDUCING_FACTOR
    global ENC_PULSES_PER_REV
    global NOM_DIAMETER
    global WHEELBASE
    global N_TURNS
    global PULSES_PER_REV
    global MM_TO_PULSES
    global ESTIMATED_PULSES_PER_TURN


    global TURN_R_WHEEL_STOPPED
    global TURN_L_WHEEL_STOPPED

    user_input = input("Number of turns for the calibration: ")
    if user_input != '':
        N_TURNS = int(user_input)

    user_input = input("Diameter of the wheels in mm: ")
    if user_input != '':
        NOM_DIAMETER = float(user_input)

    user_input = input("Reducing factor: ")
    if user_input != '':
        REDUCING_FACTOR = float(user_input)

    user_input = input("Encoder pulses per revolution: ")
    if user_input != '':
        ENC_PULSES_PER_REV = float(user_input)

    PULSES_PER_REV = REDUCING_FACTOR * ENC_PULSES_PER_REV
    MM_TO_PULSES = (pi * NOM_DIAMETER) / PULSES_PER_REV

    user_input = input("Wheelbase: ")
    if user_input != '':
        WHEELBASE = float(user_input)
    circumference = N_TURNS * 2 * pi * WHEELBASE / 10

    if log10(circumference) >= 4: #5-digit number!
        circumference = str(9999)
    elif log10(circumference) > 3: #4-digit number!
        circumference= str(int(circumference))
    elif log10(circumference) > 2: #3-digit number!
        circumference = '0' + str(int(circumference)) #We have to round it. I prefer this instead of floor()
    elif log10(circumference) > 1: #2-digit number
        circumference = '00' + str(int(circumference))
    elif log10(circumference) > 0: #2-digit number
        circumference = '000' + str(int(circumference))

    TURN_L_WHEEL_STOPPED = "C000" + circumference
    TURN_R_WHEEL_STOPPED = "D000" + circumference

    ESTIMATED_PULSES_PER_TURN = (2 * pi * WHEELBASE) / (pi * NOM_DIAMETER) * PULSES_PER_REV

    return

def print_updated_data():
    cont = 'n'

    print("Number of turns for the calibration: %d\n" % (N_TURNS), end="")
    print("Arduino baudrate: %d\n" % (ARDUINO_BAUDRATE), end="")
    print("Reducing factor: %g\n" % (REDUCING_FACTOR), end="")
    print("Eencoder pulses per rev: %d\n" % (ENC_PULSES_PER_REV), end="")
    print("Nominal diameter: %g\n" % (NOM_DIAMETER), end="")
    print("Wheelbase: %g\n" % (WHEELBASE), end="")
    print("Number of turns to perform: %d\n" % (N_TURNS), end="")
    print("Diameter error (ED): %g\n" % (ED), end="")
    print("Scaling error (ES): %g\n" % (ES), end="")
    print("K factor: %g\n" % (K), end="")
    print("Pulses per rev: %g\n" % (PULSES_PER_REV), end="")
    print("Mm to pulses conversion: %g\n" % (MM_TO_PULSES), end="")
    print("Est. pulses per turn: %g\n" % (ESTIMATED_PULSES_PER_TURN), end="")
    print("Real pulses per turn: %g\n" % (REAL_PULSES_PER_TURN), end="")
    print("Turn L stopped command: %s\n" % (TURN_L_WHEEL_STOPPED), end="")

    while cont != 'Y':
        cont = input("Proceed with these results? (Y/n)")
        if cont == 'n':
            exit()
    return

def main():

    #initial_data()
    #print_updated_data()
    arduino = find_N_open_serial_port()
    #execute_command("Straight 3 m", arduino)

    #execute_command("Turn L stopped", arduino)

    #data_file = execute_command("Get data", arduino)

    #print_file(data_file)

    arduino.open()

    time.sleep(1)

    arduino.write("50500".encode())

    time.sleep(1)

    arduino.write("40500".encode())

    #time.sleep(5)

    arduino.close()

main()
