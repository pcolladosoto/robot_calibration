#Imports
import glob #For finding the serial terminal
import serial #For working witht the serial ports
import os #For system calls to ls and rm
from numpy import convolve #For the convolution
from re import findall #For parsing the input strings
from math import pi #We need our friend PI

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
ERROR_MESSAGES = ["PORT NOT FOUND, ABORTING!",
                  "SIGNAL LENGTHS ARE DIFFERENT, ABORTING!"]
PORT_NOT_FOUND = 0
ERROR_SIGNAL_TREATMENT = 1

#Commands
GET_DATA = 'F\n'
TURN_L_WHEEL_STOPPED = 'C000XXX\n' #XXX is the distance in cm
TURN_R_WHEEL_STOPPED = 'D000XXX\n' #XXX is the distance in cm
TURN_BOTH_WHEELS = '3XXX\n' #XXX is (I guess) the angle to turn in deg
GO_STRAIGHT_3M = '43000\n' #XXXX is the distance in mm

def find_N_open_serial_port():
    #Expand all the paths in the argument. [A-Za-z] matches any letter both upper and lower case. Its a RegExpr
    #glob.glob("/dev/tty[A-Za-z]*")
    #Maybe this'll work too
    candidates = glob.glob('/dev/tty[A-Za-z]*')

    #open_ports = []
    #Try to open each ttyUSBX port. If it's not being used, throw the exception and continue
    for path in candidates:
        try:
            #port = serial.Serial(path, ARDUINO_BAUDRATE, timeout = None)
            port = serial.Serial(path, timeout = None)
            if check_if_arduino():
                print("Found Arduino at port %s\n" % (port.name), end="")
                port.close()
                return port
            else:
                port.close()
                #port.__del__() #It should't be needed though!
            #open_ports.append(port)
        except(OSError, serial.SerialException):
            pass

    #return open_ports
    return PORT_NOT_FOUND

def check_if_arduino():
    #Kind of hacky... Write the output of the given folder to tmp and read from it to see if it's an Arduino. If ls returns a non-zero value we have an error!
    if os.system("ls /dev/serial/by-id > tmp") != 0 or not ('Arduino' in open('tmp', 'r').read()):
        os.system("rm tmp")
        return False
    os.system("rm tmp")
    return True

def get_data(port):
    dumped_data = open("dumped_data", "w+")
    port.write(GET_DISTANCES)

    while port.in_waiting > 0:
        dumped_data.write(port.read())

    dumped_data.seek(0)

    return dumped_data

def read_N_parse_data(dumped_data):
    for line in dumped_data:
        extract_data(line)
    return

def extract_data(string_to_parse, pulses_array, distances_array):
    #r denotes a raw string so that we have no problems with the '\' char. \d matches any one digit and + let's it get any number of digits. It's like [0-9]. The function returns a list with both numbers! They are still strings, so we need to cast them to int... Even though Arduino is giving us chars, Python doesn't have this type... We have to use ints even if it is kind of overkill...
    matches = findall(r"\d+", string_to_parse)
    pulses_array.append(matches[0])
    distances_array.append(matches[1])

def populate_signal(pulses_array, distances_array):
    original_index = 0
    current_pulses = pulses_array[original_index]
    current_distance = distances_array[original_index]
    populated_array = []

    for index in range(pulses_array[-1]):
        populated_array.append(current_distance)
        if index > current_pulses:
            original_index += 1
            current_pulses = pulses_array[original_index]
            if current_distance > distances_array[original_index]:
                current_distance = distances_array[original_index] + 256 #Note how 250 + 10 = 260, but 250 + 6 + 4 = 0 + 4 = 4 and 4 + 255 = 259!!
            else:
                current_distance = distances_array[original_index]

    return populated_array

def signal_reversal(input_signal):
    return input_signal.reverse() #Returns none, but the signal has been reversed!

def convolution_time(original_signal, reversed_signal):
    if len(original_signal) != len(reversed_signal):
        return ERROR_SIGNAL_TREATMENT
    return convolve(original_signal, reversed_signal)

def find_maximum(convolved_signal):
    length_convolved = len(convolved_signal)
    lower_limit = length_convolved / 2 - length_convolved / (2 * N_TURNS) - ESTIMATED_PULSES_PER_TURN / 0.1
    upper_limit = length_convolved / 2 - length_convolved / (2 * N_TURNS) + ESTIMATED_PULSES_PER_TURN / 0.1

    current_max = convolved_signal[lower_limit]
    current_max_index = lower_limit

    for index in range(lower_limit + 1, upper_limit):
        if convolved_signal[index] > current_max:
            current_max = convolved_signal[index]
            current_max_index = index

    return length_convolved / 2 + 0.5 - current_max_index #It's equivalent to subtracting the index from the signal length!

def execute_command(command, port):
    finished = 0

    if command == "Get data":
        get_data(port)
    elif command == "Turn L stopped":
        port.write(TURN_L_WHEEL_STOPPED)
        while True:
            if port.read() == '.':
                finished += 1
                if finished == 2:
                    break
    elif command == "Turn R stopped":
        port.write(TURN_R_WHEEL_STOPPED)
        while True:
            if port.read() == '.':
                finished += 1
                if finished == 2:
                    break
    elif command == "Turn both wheels":
        port.write(TURN_BOTH_WHEELS)
        while True:
            if port.read() == '.':
                finished += 1
                if finished == 2:
                    break
    elif command == "Straight 3 m":
        port.write(GO_STRAIGHT_3M)
        while True:
            if port.read() == '.':
                finished += 1
                if finished == 2:
                    break
    return

def initial_data():
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

    ESTIMATED_PULSES_PER_TURN = (2 * pi * WHEELBASE) / (pi * NOM_DIAMETER) * PULSES_PER_REV

    return

def main():
    pulses_array = []
    distances_array = []

    arduino_port = find_N_open_serial_port()

    if arduino == PORT_NOT_FOUND:
        print(ERROR_MESSAGES[PORT_NOT_FOUND], end="")
        exit()

    #We will perform each test 3 times to acquire the best possible calibration
    for i in range(3):
        execute_command("Turn L stopped")
        execute_command("Get data")
