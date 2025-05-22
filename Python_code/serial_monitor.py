#This file reads the activity in the serial monitor and creates a csv file with the states.
#Author: Sam Farkhooi

import serial

port = '/dev/cu.usbserial-0001'
baud = 115200

with serial.Serial(port, baud, timeout=1) as ser, open('states.csv', 'w') as f:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print(line)
            f.write(line + '\n')
