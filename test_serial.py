#!/usr/bin/python3.5
from time import sleep
import serial

with serial.Serial('/dev/ttyUSB0', 115200) as ser:
    print(ser)
    while True:
        ser.write(b"hello")
        print(ser.readline()[:-1].decode())
