#! /usr/bin/env python3

import serial
import random
import string
import time

def randomString(stringLength):
    letters = string.ascii_letters
    return ''.join(random.choice(letters) for i in range(stringLength))

time_elapsed = 0
errors = 0
num_packets = 1000
ser = serial.Serial('/dev/ttyACM0', 115200)

for i in range(num_packets):

    test_string = randomString(1000) + '\n'

    start_time = time.time()

    ser.write(str.encode(test_string))
    s = ser.readline()
    ser.readline()

    stop_time = time.time()

    for s, t in zip(s.decode('utf-8'), test_string):
        if (s != t):
            print("Error in transmission")
            errors += 1

    time_elapsed += (stop_time - start_time)
    print("Time : ", stop_time - start_time)

    time.sleep(0.001)

print("avg time :", time_elapsed/num_packets)
print("total number of errors : ", errors)
