#Imports
import glob, serial, os, time, signal
#import sys
from numpy import convolve
from re import findall
from math import pi, log10
from matplotlib import pyplot as plotter

#Colors
terminal_colors = {
    "end_color": "\033[0m",
    "lime": "\033[38;2;0;255;0m",
    "red": "\033[38;2;255;0;0m",
    "cyan": "\033[38;2;0;255;255m",
    "pink": "\033[38;2;255;105;180m"
}

#Constants
CONSTANTS = {
    "ARDUINO_BAUDRATE": 38400,
    "REDUCING_FACTOR": 50.9,
    "ENC_PULSES_PER_REV": 7,
    "NOM_DIAMETER": 190,
    "WHEELBASE": 535,
    "N_TURNS": 8,
    "N_REPS": 1,
    "ED": 1,
    "ES": 1,
    "DR": -1,
    "K": -1,
    "PULSES_PER_REV": 356.3,
    "MM_TO_PULSES": 1.675281,
    "ESTIMATED_PULSES_PER_TURN": 2006.53,
    "REAL_PULSES_PER_TURN_L": -1,
    "REAL_PULSES_PER_TURN_R": -1,
    "REAL_PULSES_PER_TURN_BOTH": -1,
    "NOISE_THRESHOLD": 100,
    "WINDOW_LIMITS": 0.1,
}

#Errors
ERROR_MESSAGES = {
    "PORT_NOT_FOUND": "PORT NOT FOUND, ABORTING!",
    "ERROR_SIGNAL_TREATMENT": "SIGNAL LENGTHS ARE DIFFERENT, ABORTING!",
    "PORT_NOT_REACHABLE": "PORT NOT REACHABLE"
}
#Commands
COMMANDS = {
    "GET_DATA": 'F',
    "TURN_L_WHEEL_STOPPED": 'C0002689',
    "TURN_R_WHEEL_STOPPED": 'D0002689',
    "TURN_BOTH_WHEELS": '3360',
    "GO_STRAIGHT_3M": '63000220',
    "GO_STRAIGHT_1M": '41000',
    "TURN_L_LITTLE": '2020',
    "TURN_R_LITTLE": '3020'
}

#Program switches
SWITCHES = {
    "VERBOSE": True,
    "DBG": True,
    "COMPUTE_L_STOPPED": True,
    "COMPUTE_R_STOPPED": True,
    "COMPUTE_BOTH": True,
    "COMPUTE_STRAIGHT": True,
    "PHASES_23_DONE": False,
    "PHASES_12_DONE": False
}

def IRQ_setup():
    return signal.signal(signal.SIGINT, handler)

def handler():
    print(terminal_colors["red"] + "Closing on received interrupt!" + terminal_colors["end_color"])
    exit()

def find_N_open_serial_port():
    candidates = glob.glob('/dev/tty[A-Za-z]*')
    for path in candidates:
        try:
            port = serial.Serial(path, baudrate = CONSTANTS["ARDUINO_BAUDRATE"], timeout = None)
            if check_if_arduino():
                print(terminal_colors["lime"] + "Found Arduino at port %s" % (port.name) + terminal_colors["end_color"])
                time.sleep(1)
                return port
            else:
                port.close()
        except(OSError, serial.SerialException):
            pass

    print(ERROR_MESSAGES["PORT_NOT_FOUND"])
    return exit()

def check_if_arduino():
    os.system("ls /dev/serial/by-id > tmp")
    if "USB2.0-Serial-if00" in open("tmp", "r").read():
        os.system("rm tmp")
        return True
    else:
        os.system("rm tmp")
        return False

def get_data(port, f_name):
    port.reset_input_buffer()
    dumped_data = open(f_name + ".txt", "w+")
    port.write(COMMANDS["GET_DATA"].encode())

    newline = ""
    while newline != "END\r\n":
        newline = (port.readline()).decode()
        dumped_data.write(newline)
    dumped_data.seek(0)

    return dumped_data

def get_straight_data(port):

    matches = findall(r"\d+", port.readline().decode())
    return int(matches[4])

def execute_command(command, port, f_name = None):
    finished = 0
    str_p = 0

    if command[0] == "S":
        port.write(command.encode())

    elif command == "Get data":
        print(terminal_colors["cyan"] + "Processing data..." + terminal_colors["end_color"])
        return get_data(port, f_name)

    elif command == "Turn L stopped":
        port.write(COMMANDS["TURN_L_WHEEL_STOPPED"].encode())
        port.reset_input_buffer()

    elif command == "Turn R stopped":
        port.write(COMMANDS["TURN_R_WHEEL_STOPPED"].encode())
        port.reset_input_buffer()

    elif command == "Turn both wheels":
        port.write(COMMANDS["TURN_BOTH_WHEELS"].encode())
        port.reset_input_buffer()

    elif command == "Advance 3 m":
        port.write(COMMANDS["GO_STRAIGHT_3M"].encode())
        port.reset_input_buffer()
        str_p = get_straight_data(port)

    elif command == "A":
        port.write(COMMANDS["TURN_L_LITTLE"].encode())

    elif command == "D":
        port.write(COMMANDS["TURN_R_LITTLE"].encode())

    else:
        return

    while True:
        char = port.read().decode()
        if char == ".":
            finished += 1
            if finished == 2:
                return str_p

def initial_data(port):

    global CONSTANTS, COMMANDS, SWITCHES

    user_input = "X"

    while user_input != "n":
        user_input = input("Accept defaults?  (Y/n) -> ")
        if user_input == "Y":
            return execute_command("S" + str(CONSTANTS["WHEELBASE"]) + "W" + str(CONSTANTS["MM_TO_PULSES"]) + "M" + str(CONSTANTS["N_TURNS"]) + "R", port)

    user_input = input("Number of turns for the calibration: ")
    if user_input != '':
        CONSTANTS["N_TURNS"] = int(user_input)

    user_input = input("Number of repetitions for the calibration: ")
    if user_input != '':
        CONSTANTS["N_REPS"] = int(user_input)

    user_input = input("Diameter of the wheels in mm: ")
    if user_input != '':
        CONSTANTS["NOM_DIAMETER"] = float(user_input)

    user_input = input("Reducing factor: ")
    if user_input != '':
        CONSTANTS["REDUCING_FACTOR"] = float(user_input)

    user_input = input("Encoder pulses per revolution: ")
    if user_input != '':
        CONSTANTS["ENC_PULSES_PER_REV"] = float(user_input)

    CONSTANTS["PULSES_PER_REV"] = CONSTANTS["REDUCING_FACTOR"] * CONSTANTS["ENC_PULSES_PER_REV"]
    CONSTANTS["MM_TO_PULSES"] = (pi * CONSTANTS["NOM_DIAMETER"]) / CONSTANTS["PULSES_PER_REV"]

    user_input = input("Wheelbase: ")
    if user_input != '':
        CONSTANTS["WHEELBASE"] = float(user_input)

    user_input = input("Do you want verbose (printing data) operation? (Y/n) -> ")
    if user_input == "n":
        SWITCHES["VERBOSE"] = False

    circumference = CONSTANTS["N_TURNS"] * 2 * pi * CONSTANTS["WHEELBASE"] / 10

    if log10(circumference) >= 4:
        circumference = str(9999)
    elif log10(circumference) > 3:
        circumference= str(int(circumference))
    elif log10(circumference) > 2:
        circumference = '0' + str(int(circumference))
    elif log10(circumference) > 1:
        circumference = '00' + str(int(circumference))
    elif log10(circumference) > 0:
        circumference = '000' + str(int(circumference))

    COMMANDS["TURN_L_WHEEL_STOPPED"] = "C000" + circumference
    COMMANDS["TURN_R_WHEEL_STOPPED"] = "D000" + circumference

    CONSTANTS["ESTIMATED_PULSES_PER_TURN"] = (2 * pi * CONSTANTS["WHEELBASE"]) / (pi * CONSTANTS["NOM_DIAMETER"]) * CONSTANTS["PULSES_PER_REV"]

    execute_command("S" + str(CONSTANTS["WHEELBASE"]) + "W" + str(CONSTANTS["MM_TO_PULSES"]) + "M" + str(CONSTANTS["N_TURNS"]) + "R", port)

    return os.system("clear")

def print_updated_data():
    cont = 'n'

    print(terminal_colors["lime"] + "The data we will procced with is:" + terminal_colors["end_color"])

    print("\tNumber of turns for the calibration: %d" % (CONSTANTS["N_TURNS"]))
    print("\tNumber of repetitions for the calibration: %d" % (CONSTANTS["N_REPS"]))
    print("\tNominal diameter: %g" % (CONSTANTS["NOM_DIAMETER"]))
    print("\tWheelbase: %g" % (CONSTANTS["WHEELBASE"]))
    print("\tMm to pulses conversion: %g" % (CONSTANTS["MM_TO_PULSES"]))
    print("\tEst. pulses per turn: %g" % (CONSTANTS["ESTIMATED_PULSES_PER_TURN"]))

    while cont != 'Y':
        cont = input("Proceed with these results? (Y/n) -> ")
        if cont == 'n':
            exit()
    return os.system("clear")

def user_tweaks(mode):
    ord = 'X'

    if mode == 'Continue?':
        while ord != '':
            ord = input(terminal_colors["cyan"] + "Press ENTER to continue: " + terminal_colors["end_color"])
    return os.system("clear")

def read_N_parse(data, p_array, d_array):
    for line in data:
        if not "Counter" in line and not "END" in line:
            extract_data(line, p_array, d_array)
    data.close()

def extract_data(string_to_parse, pulses_array, distances_array):
    matches = findall(r"\d+", string_to_parse)
    pulses_array.append(int(matches[0]))
    distances_array.append(int(matches[1]))

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
            if distances_array[i] < CONSTANTS["NOISE_THRESHOLD"]:
                populated_array.append(distances_array[i])
            index += 1
        last_x = x
        i += 1

    return populated_array

def convolution_time(original_signal, reversed_signal):
    if len(original_signal) != len(reversed_signal):
        return print(ERROR_MESSAGES["ERROR_SIGNAL_TREATMENT"])
    return convolve(original_signal, reversed_signal)

def find_maximum(convoluted_signal, mode):
    length_convoluted = len(convoluted_signal)
    if mode == "Stopped":
        lower_limit = int(length_convoluted / 2 - length_convoluted / (2 * CONSTANTS["N_TURNS"]) - CONSTANTS["ESTIMATED_PULSES_PER_TURN"] * CONSTANTS["WINDOW_LIMITS"])
        upper_limit = int(length_convoluted / 2 - length_convoluted / (2 * CONSTANTS["N_TURNS"]) + CONSTANTS["ESTIMATED_PULSES_PER_TURN"] * CONSTANTS["WINDOW_LIMITS"])
    else:
        lower_limit = int(length_convoluted / 2 - length_convoluted / (2 * CONSTANTS["N_TURNS"]) - (CONSTANTS["ESTIMATED_PULSES_PER_TURN"] / 2) * CONSTANTS["WINDOW_LIMITS"])
        upper_limit = int(length_convoluted / 2 - length_convoluted / (2 * CONSTANTS["N_TURNS"]) + (CONSTANTS["ESTIMATED_PULSES_PER_TURN"] / 2) * CONSTANTS["WINDOW_LIMITS"])

    current_max = convoluted_signal[lower_limit]
    current_max_index = lower_limit

    for index in range(lower_limit + 1, upper_limit):
        if convoluted_signal[index] > current_max:
            current_max = convoluted_signal[index]
            current_max_index = index

    if SWITCHES["VERBOSE"]:
        os.system("clear")
        print(terminal_colors["pink"] + "Obtained results:" + terminal_colors["end_color"])
        print("\tLower limit: %d" % (lower_limit))
        print("\tUpper limit: %d" % (upper_limit))
        print("\tAbsolute MAX at: %d" % (length_convoluted / 2))
        print("\tMax at: %d" % (current_max_index))

    return length_convoluted / 2 + 0.5 - current_max_index #It's equivalent to subtracting the index from the signal length!

def show_signal(input_signal, y_label, title):
    plotter.ylabel(y_label)
    plotter.xlabel("Pulses")
    plotter.title(title)
    plotter.plot(input_signal)
    plotter.show()

    return

def main():
    global CONSTANTS

    p_array, d_array, L_PULSES, R_PULSES, B_PULSES = [], [], [], [], []

    print(terminal_colors["lime"] + "Beginning calibration!\n" + terminal_colors["end_color"])
    print(terminal_colors["cyan"] + "The calibration procedure is composed of 4 steps:" + terminal_colors["end_color"])
    print("\t1. The robot will turn with the left wheel stopped for the specified number of turns.")
    print("\t2. The robot will turn with the right wheel stopped for the specified number of turns.")
    print("\t3. The robot will turn with both wheels for the specified number of turns.")
    print("\t4. The robot will go straight for 3 meters. You will have to measure the actual traversed distance.\n")
    print("If you don't accept the defaults please refer to the README.md in the GitHub repo.\n")

    user_tweaks("Continue?")

    arduino = find_N_open_serial_port()

    user_tweaks("Continue?")

    initial_data(arduino)
    print_updated_data()

    print(terminal_colors["lime"] + "Beginning phase 1: Turning with the left wheel stopped" + terminal_colors["end_color"])

    for i in range(CONSTANTS["N_REPS"]):
        if i == 0:
            if input("Input S to skip the test: ") == "S":
                SWITCHES["COMPUTE_L_STOPPED"] = False
                continue
        p_array, d_array = [], []
        execute_command("Turn L stopped", arduino)
        data_file = execute_command("Get data", arduino, f_name = "L_stopped_" + str(i))
        read_N_parse(data_file, p_array, d_array)
        show_signal(d_array, "Distance [cm]", "L Stopped RAW Distances")
        baked_signal = populate_signal(p_array, d_array)
        show_signal(baked_signal, "Distance [cm]", "L Stopped Populated Signal")
        og_signal = baked_signal.copy()
        baked_signal.reverse()
        convoluted_signal = convolution_time(og_signal, baked_signal)
        show_signal(convoluted_signal, "Surface [cm * cm]", "L Stopped Convolution")
        L_PULSES.append(find_maximum(convoluted_signal, "Stopped"))
        if SWITCHES["VERBOSE"]:
            print("\tComputed pulses: %g" % (L_PULSES[i]))

    user_tweaks("Continue?")

    print(terminal_colors["lime"] + "Beginning phase 2: Turning with the right wheel stopped" + terminal_colors["end_color"])

    for i in range(CONSTANTS["N_REPS"]):
        if i == 0:
            if input("Input S to skip the test: ") == "S":
                SWITCHES["COMPUTE_R_STOPPED"] = False
                continue
        p_array, d_array = [], []
        execute_command("Turn R stopped", arduino)
        data_file = execute_command("Get data", arduino, f_name = "R_stopped_" + str(i))
        read_N_parse(data_file, p_array, d_array)
        show_signal(d_array, "Distance [cm]", "R Stopped RAW Distances")
        baked_signal = populate_signal(p_array, d_array)
        show_signal(baked_signal, "Distance [cm]", "R Stopped Populated Signal")
        og_signal = baked_signal.copy()
        baked_signal.reverse()
        convoluted_signal = convolution_time(og_signal, baked_signal)
        show_signal(convoluted_signal, "Surface [cm * cm]", "R Stopped Convolution")
        R_PULSES.append(find_maximum(convoluted_signal, "Stopped"))
        if SWITCHES["VERBOSE"]:
            print("\tComputed pulses: %g" % (R_PULSES[i]))

    user_tweaks("Continue?")

    print(terminal_colors["lime"] + "Beginning phase 3: Turning with both wheels" + terminal_colors["end_color"])

    for i in range(CONSTANTS["N_REPS"]):
        if i == 0:
            if input("Input S to skip the test: ") == "S":
                SWITCHES["COMPUTE_BOTH"] = False
                continue
        p_array, d_array = [], []
        execute_command("Turn both wheels", arduino)
        data_file = execute_command("Get data", arduino, f_name = "Both_on_" + str(i))
        read_N_parse(data_file, p_array, d_array)
        show_signal(d_array, "Distance [cm]", "Both On RAW Distances")
        baked_signal = populate_signal(p_array, d_array)
        show_signal(baked_signal, "Distance [cm]", "Both On Populated Signal")
        og_signal = baked_signal.copy()
        baked_signal.reverse()
        convoluted_signal = convolution_time(og_signal, baked_signal)
        show_signal(convoluted_signal, "Surface [cm * cm]", "Both On Convolution")
        B_PULSES.append(find_maximum(convoluted_signal, "Both"))
        if SWITCHES["VERBOSE"]:
            print("\tComputed pulses: %g" % (B_PULSES[i]))

    user_tweaks("Continue?")

    if SWITCHES["COMPUTE_L_STOPPED"]:
        CONSTANTS["REAL_PULSES_PER_TURN_L"] = int(sum(L_PULSES) / len(L_PULSES))

    if SWITCHES["COMPUTE_R_STOPPED"]:
        CONSTANTS["REAL_PULSES_PER_TURN_R"] = int(sum(R_PULSES) / len(R_PULSES))

    if SWITCHES["COMPUTE_BOTH"]:
        CONSTANTS["REAL_PULSES_PER_TURN_BOTH"] = int(sum(B_PULSES) / len(B_PULSES))

    print(terminal_colors["cyan"] + "Computed pulses per turn:" + terminal_colors["end_color"])
    print(terminal_colors["pink"] + "\tLeft pulses: %g" % (CONSTANTS["REAL_PULSES_PER_TURN_L"]))
    print("\tRight pulses: %g" % (CONSTANTS["REAL_PULSES_PER_TURN_R"]))
    print("\tBoth pulses: %g" % (CONSTANTS["REAL_PULSES_PER_TURN_BOTH"]))

    print(terminal_colors["cyan"] + "K and ED values:" + terminal_colors["end_color"])

    if SWITCHES["COMPUTE_R_STOPPED"]:
        if SWITCHES["COMPUTE_BOTH"]:
            SWITCHES["PHASES_23_DONE"] = True
            CONSTANTS["K"] = 2 * CONSTANTS["REAL_PULSES_PER_TURN_BOTH"] / CONSTANTS["REAL_PULSES_PER_TURN_R"]
            print("\tK value: %g" % (CONSTANTS["K"]))
        if SWITCHES["COMPUTE_L_STOPPED"]:
            SWITCHES["PHASES_12_DONE"] = True
            CONSTANTS["ED"] = CONSTANTS["REAL_PULSES_PER_TURN_R"] / CONSTANTS["REAL_PULSES_PER_TURN_L"]
            print("\tDiameter error (ED): %g" % (CONSTANTS["ED"]) + terminal_colors["end_color"])
            execute_command("S" + str(CONSTANTS["ED"]) + "D", arduino)

    user_tweaks("Continue?")

    print(terminal_colors["lime"] + "Beginning phase 4: Going straight for 3 meters" + terminal_colors["end_color"])

    if input("Input S to skip the test: ") != "S":
        straight_pulses = execute_command("Advance 3 m", arduino)
        print(terminal_colors["pink"] + "Recorded pulses: %d" % (straight_pulses) + terminal_colors["end_color"])
        real_distance = float(input("Please input the traversed distance in mm: "))
        CONSTANTS["MM_TO_PULSES"] = real_distance / straight_pulses
        print(terminal_colors["pink"] + "MM -> Pulses conversion factor: %g" % (CONSTANTS["MM_TO_PULSES"]))
        execute_command("S" + str(CONSTANTS["MM_TO_PULSES"]) + "M", arduino)
    else:
        SWITCHES["COMPUTE_STRAIGHT"] = False

    if SWITCHES["COMPUTE_STRAIGHT"]:
        CONSTANTS["DR"] = CONSTANTS["MM_TO_PULSES"] * CONSTANTS["PULSES_PER_REV"] / pi
        print("Real right wheel diameter: %g" % (CONSTANTS["DR"]))

        if SWITCHES["PHASES_23_DONE"]:
            CONSTANTS["WHEELBASE"] = CONSTANTS["REAL_PULSES_PER_TURN_R"] * CONSTANTS["MM_TO_PULSES"] * CONSTANTS["K"] / (2 * pi)
            print("Adjusted wheelbase: %g mm" % (CONSTANTS["WHEELBASE"]))
            execute_command("S" + str(CONSTANTS["WHEELBASE"]) + "w", arduino)

        if SWITCHES["PHASES_12_DONE"]:
            CONSTANTS["ES"] = (CONSTANTS["DR"] / CONSTANTS["NOM_DIAMETER"]) * ((1 + CONSTANTS["ED"]) / 2)
            print("Scaling error (ES): " % (CONSTANTS["ES"]) + terminal_colors["end_color"])

    execute_command("S" + "1R", arduino)

    user_input = input("Would you like a final summary of the computed data? (Y/n) -> ")
    if user_input == "Y":
        for key, value in CONSTANTS.items():
            print ("Constant: %s " % (key) + terminal_colors["cyan"] + "has a value of %g " % (value) + terminal_colors["end_color"])

    print(terminal_colors["lime"] + "\nCalibration finished! Thanks for using our tool!" + terminal_colors["end_color"])

    if not SWITCHES["DBG"]:
        os.system("rm *.txt")

    exit()

if __name__ == "__main__":
    main()
