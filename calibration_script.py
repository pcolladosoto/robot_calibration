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
WHEELBASE = 535 #In mm
N_TURNS = 8
N_REPS = 3
ED = 1
ES = 1
K = -1 #Computed later on
PULSES_PER_REV = -1 #Computed later on
MM_TO_PULSES = -1 #Computed later on
ESTIMATED_PULSES_PER_TURN = -1 #Computed later on
REAL_PULSES_PER_TURN_L = -1 #Computed later on
REAL_PULSES_PER_TURN_R = -1
REAL_PULSES_PER_TURN_BOTH = -1

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
TURN_L_LITTLE = '2020'
TURN_R_LITTLE = '3020'

def find_N_open_serial_port():
    #Expand all the paths in the argument. [A-Za-z] matches any letter both upper and lower case. Its a RegExpr
    #glob.glob("/dev/tty[A-Za-z]*")
    #Maybe this'll work too
    candidates = glob.glob('/dev/tty[A-Za-z]*')

    #Try to open each ttyUSBX port. If it's not being used, throw the exception and continue
    for path in candidates:
        print("Checking: %s\n" % (path), end="")
        try:
            port = serial.Serial(path, baudrate = ARDUINO_BAUDRATE, timeout = None)
            if check_if_arduino():
                print("Found Arduino at port %s\n" % (port.name), end="")
                time.sleep(1)
                return port
            else:
                port.close()
        except(OSError, serial.SerialException):
            print("%s" % (ERROR_MESSAGES[PORT_NOT_REACHABLE]), end="")
            pass

    #return open_ports
    return exit()

def check_if_arduino():
    #Kind of hacky... Write the output of the given folder to tmp and read from it to see if it's an Arduino. If ls returns a non-zero value we have an error! Our Arduino is not original, so we have to look for another string...
    if os.system("ls /dev/serial/by-id > tmp") != 0 or not ('USB2.0-Serial-if00' in open('tmp', 'r').read()):
        os.system("rm tmp")
        return False
    os.system("rm tmp")
    return True

def get_data(port):
    port.reset_input_buffer()
    dumped_data = open("data.txt", "w+")
    port.write("F".encode())

    newline = ""
    while newline != "END\r\n":
        newline = (port.readline()).decode()
        dumped_data.write(newline)
    dumped_data.seek(0)

    return dumped_data

def get_straight_data(port):
    matches = findall(r"\d+", port.readline().decode())

    return matches[4]

def print_file(data):
    data = open("data.txt", "r")

    for line in data:
        print(line, end = "")

    data.close()

    return

def execute_command(command, port):
    finished = 0

    if command == "Get data":
        print("Let's go get that data!\n", end="")
        return get_data(port)

    elif command == "Turn L stopped":
        print("Turning with the L wheel stopped!\n", end="")
        port.write(TURN_L_WHEEL_STOPPED.encode())
        port.reset_input_buffer()

    elif command == "Turn R stopped":
        print("Turning with the R wheel stopped!\n", end="")
        port.write(TURN_R_WHEEL_STOPPED.encode())
        port.reset_input_buffer()

    elif command == "Turn both wheels":
        for i in range(N_TURNS):
            print("Turning with both wheels!\n", end="")
            port.write(TURN_BOTH_WHEELS.encode())
            port.reset_input_buffer()

    elif command == "Straight 3 m":
        print("Going straight for 3 m!\n", end="")
        port.write(GO_STRAIGHT_3M.encode())

    elif command == "A":
        port.write(TURN_L_LITTLE.encode())

    elif command == "D":
        port.write(TURN_R_LITTLE.encode())

    elif command[0] == "S":
        port.write(command.encode())

    while True:
        char = port.read().decode()
        #print("%s" % (char), end="")
        #sys.stdout.flush()
        if char == ".":
            print("Hey!\n", end="")
            finished += 1
            if finished == 2:
                return

def initial_data():

    global REDUCING_FACTOR, ENC_PULSES_PER_REV, NOM_DIAMETER, WHEELBASE, N_TURNS, PULSES_PER_REV, MM_TO_PULSES, ESTIMATED_PULSES_PER_TURN, TURN_R_WHEEL_STOPPED, TURN_L_WHEEL_STOPPED

    user_input = input("Number of turns for the calibration: ")
    if user_input != '':
        N_TURNS = int(user_input)

    user_input = input("Number of repetitions for the calibration: ")
    if user_input != '':
        N_REPS = int(user_input)

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

    execute_command("S" + str(WHEELBASE) + "W" + str(MM_TO_PULSES) + "M" + str(NOM_DIAMETER) + "D")

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
    print("Turn L stopped command: %s\n" % (TURN_L_WHEEL_STOPPED), end="")

    while cont != 'Y':
        cont = input("Proceed with these results? (Y/n)")
        if cont == 'n':
            exit()
    return

def user_tweaks(port, mode):
    ord = 'X'

    if mode == 'continue?':
        while ord != '':
            ord = input("Press ENTER to continue\n", end = "")
    else:
        print("Use the A and D keys + ENTER to correct the orientation of the robot. ", end = "")
        print("You can also move the robot manually, but it's kind of heavy... ", end = "")
        print("Press ENTER when finished.", end = "")

        while ord != '':
            ord = input("Command: ")
            execute_command(ord.upper(), port)

    return

def read_N_parse(data, p_array, d_array):
    for line in data:
        if not "Counter" in line and not "END" in line:
            extract_data(line, p_array, d_array)
    data.close()

def extract_data(string_to_parse, pulses_array, distances_array):
    matches = findall(r"\d+", string_to_parse)
    pulses_array.append(int(matches[0]))
    distances_array.append(int(matches[1]))

def print_array(array, msg):

    print(msg)
    for x in range(len(array)):
        print("Position: %d \t Distance: %d" % (x, array[x]))

    print("Length: " + str(len(array)))

    return

def populate_signal(pulses_array, distances_array):
    populated_array = []
    index = 0
    i = 0
    n_jumps = 0
    last_x = 0
    for x in pulses_array:
        if x < last_x:
            n_jumps += 1
        while index < x + n_jumps * 256:
            populated_array.append(distances_array[i])
            index += 1
        last_x = x
        i += 1

    return populated_array

def signal_reversal(input_signal):
    return input_signal.reverse() #Returns none, but the signal has been reversed!

def convolution_time(original_signal, reversed_signal):
    if len(original_signal) != len(reversed_signal):
        return ERROR_SIGNAL_TREATMENT
    return convolve(original_signal, reversed_signal)

def find_maximum(convoluted_signal, mode):
    length_convoluted = len(convoluted_signal)
    if mode == "Stopped":
        lower_limit = int(length_convoluted / 2 - length_convoluted / (2 * N_TURNS) - ESTIMATED_PULSES_PER_TURN * 0.1)
        upper_limit = int(length_convoluted / 2 - length_convoluted / (2 * N_TURNS) + ESTIMATED_PULSES_PER_TURN * 0.1)
    else:
        lower_limit = int(length_convoluted / 2 - length_convoluted / (2 * N_TURNS) - (ESTIMATED_PULSES_PER_TURN / 2) * 0.1)
        upper_limit = int(length_convoluted / 2 - length_convoluted / (2 * N_TURNS) + (ESTIMATED_PULSES_PER_TURN / 2) * 0.1)

    print("Lower limit: %d\n" % (lower_limit), end="")
    print("Upper limit: %d\n" % (upper_limit), end="")

    current_max = convoluted_signal[lower_limit]
    current_max_index = lower_limit

    for index in range(lower_limit + 1, upper_limit):
        if convoluted_signal[index] > current_max:
            current_max = convoluted_signal[index]
            current_max_index = index

    return length_convoluted / 2 + 0.5 - current_max_index #It's equivalent to subtracting the index from the signal length!

def main():
    global REAL_PULSES_PER_TURN_R, REAL_PULSES_PER_TURN_L, REAL_PULSES_PER_TURN_BOTH, K, ED, MM_TO_PULSES, WHEELBASE

    p_array = []
    d_array = []

    L_PULSES = []
    R_PULSES = []
    B_PULSES = []

    print("Beginning calibration!")

    arduino = find_N_open_serial_port()

    initial_data()

    for i in range(N_REPS):
        execute_command("Turn L stopped", arduino)
        data_file = execute_command("Get data", arduino)
        read_N_parse(data_file, p_array, d_array)
        baked_signal = populate_signal(p_array, d_array)
        og_signal = baked_signal
        rev_signal = signal_reversal(baked_signal)
        convoluted_signal = convolution_time(og_signal, reversed_signal)
        L_PULSES.append(find_maximum(convoluted_signal, "Stopped"))
        print("Computed pulses: %g\n" % (L_PULSES[i]), end = "")

    user_tweaks(arduino, "continue?")

    for i in range(N_REPS):
        p_array = []
        d_array = []
        execute_command("Turn R stopped", arduino)
        data_file = execute_command("Get data", arduino)
        read_N_parse(data_file, p_array, d_array)
        baked_signal = populate_signal(p_array, d_array)
        og_signal = baked_signal
        rev_signal = signal_reversal(baked_signal)
        convoluted_signal = convolution_time(og_signal, reversed_signal)
        R_PULSES.append(find_maximum(convoluted_signal, "Stopped"))
        print("Computed pulses: %g\n" % (R_PULSES[i]), end = "")

    user_tweaks(arduino, "continue?")

    for i in range(N_REPS):
        p_array = []
        d_array = []
        execute_command("Turn both wheels", arduino)
        data_file = execute_command("Get data", arduino)
        read_N_parse(data_file, p_array, d_array)
        baked_signal = populate_signal(p_array, d_array)
        og_signal = baked_signal
        rev_signal = signal_reversal(baked_signal)
        convoluted_signal = convolution_time(og_signal, rev_signal)
        B_PULSES.append(find_maximum(convoluted_signal, "Both"))
        print("Computed pulses: %g\n" % (B_PULSES[i]), end = "")

    REAL_PULSES_PER_TURN_L = sum(L_PULSES) / len(L_PULSES)
    REAL_PULSES_PER_TURN_R = sum(R_PULSES) / len(R_PULSES)
    REAL_PULSES_PER_TURN_BOTH = sum(B_PULSES) / len(B_PULSES)

    K = 2 * REAL_PULSES_PER_TURN_BOTH / REAL_PULSES_PER_TURN_R

    print("K value: %d pulses\n" % (K), end = "")

    ED = REAL_PULSES_PER_TURN_R / REAL_PULSES_PER_TURN_L

    print("Diameter error: %g\n" % (ED), end = "")

    user_tweaks(arduino, "Correct orientation")

    execute_command("Straight 3m", arduino)

    arduino.reset_input_buffer()

    straight_pulses = get_straight_data(arduino)

    print("Recorded pulses: %d\n" % (straight_pulses), end = "")

    real_distance = float(input("Please input the traversed distance: "))

    MM_TO_PULSES = real_distance / straight_pulses

    WHEELBASE = REAL_PULSES_PER_TURN_R * MM_TO_PULSES * K / (2 * pi)

    print("Adjusted wheelbase: %g mm\n" % (WHEELBASE))

    execute_command("S" + str(WHEELBASE) + "w" + str(ED) + "D" + str(MM_TO_PULSES) + "M", arduino)

    print("Calibration finished!\n", end = "")

    exit()

main()
