from math import pi
from numpy import convolve
from re import findall

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
    for x in array:
        print(x)

    print("Length: " + str(len(array)))

    return

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
                #Note how 250 + 10 = 260, but 250 + 6 + 4 = 0 + 4 = 4 and 4 + 255 = 259!!
                current_distance = populated_array[index - 1] + distances_array[original_index] - distances_array[original_index - 1] + 256
            else:
                current_distance = populated_array[index - 1] + distances_array[original_index] - distances_array[original_index - 1]
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

def main():
    p_array = []
    d_array = []

    dumped_data = open("data.txt", "r")

    read_N_parse(dumped_data, p_array, d_array)

    #print_array(p_array, "Pulses array: ")
    print_array(d_array, "Distances array: ")

    full_array = populate_signal(p_array, d_array)

    print_array(full_array, "Populated array: ")

main()
