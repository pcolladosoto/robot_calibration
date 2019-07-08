import serial

ard = serial.Serial("/dev/ttyUSB0", baudrate = 38400, timeout = None)

dumped_data = open("data.txt", "w+")

#ard.write("C0000500".encode())
ard.write("F".encode())

newline = ""

#Documentation tells that Serial.println() in Arduino appends BOTH a '\r' and a '\n' to the string!
while newline != "END\r\n":
    newline = (ard.readline()).decode()
    #print(newline, end = "")
    dumped_data.write(newline)

dumped_data.seek(0)

for line in dumped_data:
    print(line, end = "")

dumped_data.close()
ard.close()

exit()
