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

def read_N_parse_data(dumped_data, p_array, d_array):
    for line in dumped_data:
        if not "Counter" in line and not "END" in line:
            extract_data(line, p_array, d_array)
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
