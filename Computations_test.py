from math import pi
from numpy import convolve
from re import findall

ESTIMATED_PULSES_PER_TURN = 948
N_TURNS = 5

def read_N_parse(data, p_array, d_array):
    for line in data:
        if not "Counter" in line and not "END" in line:
            extract_data(line, p_array, d_array)

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

def find_maximum(convolved_signal):
    length_convolved = len(convolved_signal)
    lower_limit = int(length_convolved / 2 - length_convolved / (2 * N_TURNS) - ESTIMATED_PULSES_PER_TURN * 0.1)
    upper_limit = int(length_convolved / 2 - length_convolved / (2 * N_TURNS) + ESTIMATED_PULSES_PER_TURN * 0.1)

    print("Lower limit: %d\n" % (lower_limit), end="")
    print("Upper limit: %d\n" % (upper_limit), end="")

    current_max = convolved_signal[lower_limit]
    current_max_index = lower_limit

    for index in range(lower_limit + 1, upper_limit):
        if convolved_signal[index] > current_max:
            current_max = convolved_signal[index]
            current_max_index = index

    return length_convolved / 2 + 0.5 - current_max_index #It's equivalent to subtracting the index from the signal length!

def main():
    p_array = []
    d_array = []

    dumped_data = open("data.txt", "r")

    read_N_parse(dumped_data, p_array, d_array)

    print_array(d_array, "Distances array: ")

    full_array = populate_signal(p_array, d_array)

    print_array(full_array, "Populated array: ")

    non_inverted = full_array

    signal_reversal(full_array)

    convolved_signal = convolution_time(non_inverted, full_array)

    print_array(convolved_signal, "Convolved signal: ")

    REAL_PULSES_PER_TURN = find_maximum(convolved_signal)

    print("The MAX is at: %g" % (REAL_PULSES_PER_TURN))

main()
