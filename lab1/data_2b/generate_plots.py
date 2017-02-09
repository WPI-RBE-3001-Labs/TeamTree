#!/usr/bin/python3.5

import matplotlib.pyplot as plt
import numpy as np
import pandas

data = pandas.read_csv('./goto_pos_response.csv', dtype=np.float16)

position1 = []
velocity1 = [0]
acceleration1 = [0, 0]
position2 = []
velocity2 = [0]
acceleration2 = [0, 0]
last_p1 = 0
last_p2 = 0
last_v1 = 0
last_v2 = 0
i = 0
for row in data.values:
    position1.append(row[3])
    position2.append(row[4])
    v1 = row[3] - last_p1
    v2 = row[4] - last_p2

    if i > 0:
        velocity1.append(v1)
        velocity2.append(v2)

    if i > 1:
        acceleration1.append(v1 - last_v1)
        acceleration2.append(v2 - last_v2)

    last_p1 = row[3]
    last_p2 = row[4]
    last_v1 = v1
    last_v2 = v2
    i += 1

plt.figure(0)
plt.title("Arm Response to Defined Positions")
plt.xlabel("Time Steps")
plt.ylabel("Degrees")
plt.plot(range(data.shape[0]), position1)
plt.plot(range(data.shape[0]), position2)
plt.legend(['theta1', 'theta2'], loc=4)
plt.savefig("B_part_5_pos.png")

plt.figure(1)
plt.title("Arm Velocity to Defined Positions")
plt.xlabel("Time Steps")
plt.ylabel("Degrees / Sec")
plt.plot(range(data.shape[0]), velocity1)
plt.plot(range(data.shape[0]), velocity2)
plt.legend(['omega1', 'omega2'], loc=4)
plt.savefig("B_part_5_vel.png")

plt.figure(2)
plt.title("Arm Acceleration to Defined Positions")
plt.xlabel("Time Steps")
plt.ylabel("Degrees / Sec^2")
plt.plot(range(data.shape[0]), acceleration1)
plt.plot(range(data.shape[0]), acceleration2)
plt.legend(['acc1', 'acc2'], loc=4)
plt.savefig("B_part_5_acc.png")

x1 = []
x2 = []
y1 = []
y2 = []
data = pandas.read_csv('./triangle_three_points.csv', dtype=np.float16)
for row in data.values:
    x1.append(row[1])
    y1.append(row[2])

data = pandas.read_csv('./triangle_lots.csv', dtype=np.float16)
for row in data.values:
    x2.append(row[1])
    y2.append(row[2])

plt.figure(3)
plt.title("Three Point Triangle vs Many Point Triangle")
plt.scatter(x1, y1, color='r', marker='x')
plt.scatter(x2, y2, color='b', marker='o')
plt.legend(['Three Points', 'Many Points'], loc=4)

plt.savefig("B_part_6.png")
plt.show()
