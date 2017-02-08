#!/usr/bin/python3.5

import matplotlib.pyplot as plt
import pandas
import csv

plt.figure(0)
data = pandas.read_csv('./pot_angle.csv')
del data['time']
plt.title("Arm Calibration")
plt.xlabel("Time Steps")
handles = plt.plot(data)
plt.legend(handles, data.columns.values, loc=4)
plt.savefig("A_part_4_arm_calibration.png")

plt.figure(1)
data = pandas.read_csv('./step_responses_30_60_90.csv')
del data['position']
del data['setpoint']
plt.title("Step Responses for Multiple Motions")
plt.xlabel("Time Steps")
handles = plt.plot(data)
plt.legend(handles, data.columns.values)
plt.savefig("A_part_9_step_zoomed.png", loc=4)

plt.figure(2)
data = pandas.read_csv('./step_responses_30_60_90.csv')
plt.title("Step Responses for Multiple Motions")
plt.xlabel("Time Steps")
handles = plt.plot(data)
plt.legend(handles, data.columns.values, loc=4)
plt.savefig("A_part_9_step_combined.png")

plt.figure(3)
data = pandas.read_csv('./step_responses.csv')
plt.title("Response for Well Tuned PID")
plt.xlabel("Time Steps")
handles = plt.plot(data)
plt.legend(handles, data.columns.values, loc=4)
plt.savefig("A_part_8_good_pid.png")

plt.figure(4)
data = pandas.read_csv('./step_responses_low_p.csv')
plt.title("Response for Low P Term")
plt.xlabel("Time Steps")
handles = plt.plot(data)
plt.legend(handles, data.columns.values, loc=4)
plt.savefig("A_part_8_low_p.png")

plt.figure(5)
data = pandas.read_csv('./step_responses_high_i.csv')
plt.title("Response for High I Term")
plt.xlabel("Time Steps")
handles = plt.plot(data)
plt.legend(handles, data.columns.values, loc=4)
plt.savefig("A_part_8_high_i.png")
plt.show()
