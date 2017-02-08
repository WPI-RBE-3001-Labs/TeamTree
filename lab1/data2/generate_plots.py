#!/usr/bin/python3.5

import matplotlib.pyplot as plt
import pandas
import csv

data = pandas.read_csv('./pot_angle.csv')
plt.title("Arm Calibration")
plt.xlabel("Time")
plt.plot(data)
plt.show()
plt.savefig("A_part_4_arm_calibration.png")

data = pandas.read_csv('./pot_angle.csv')
plt.title("Arm Calibration")
plt.xlabel("Time")
plt.plot(data)
plt.savefig("A_part_4_arm_calibration.png")
plt.show()
