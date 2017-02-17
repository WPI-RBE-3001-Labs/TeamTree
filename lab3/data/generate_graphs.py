#!/usr/bin/python3.5

import matplotlib.pyplot as plt
from math import radians
import numpy as np
import pandas

# # histrograms of delta encoder counts
# for i, ext in enumerate(['3', '6', '-3']):
#     f = open('./encoder_ticks_%s.csv'%ext)
#
#     differences = []
#     last_datum = 0
#     for j, line in enumerate(f.readlines()):
#         datum = int(line.strip("\n"))
#         if j > 0:
#             diff = last_datum - int(datum)
#             differences.append(diff)
#         last_datum = int(datum)
#
#     plt.figure(i)
#     bins = list(np.arange(-11.5,11.5))
#     plt.hist(differences, bins)
#     plt.xlabel("Count difference")
#     plt.xlim(-11.5, 11.5)
#     plt.title("Change in Encoder Ticks at %s volts"%ext)
#     plt.savefig('encoder_ticks_%s.png' % ext)
#
# # histograms of accelerometer data
# plt.figure(3)
# plt.subplot(2,3,1)
# data = pandas.read_csv('10sec_horizontal.csv')
# data['ax'].hist(bins=8)
# plt.title('Horizontal X Acceleration')
# plt.subplot(2,3,2)
# data['ay'].hist(bins=8)
# plt.title('Horizontal Y Acceleration')
# plt.subplot(2,3,3)
# data['az'].hist(bins=8)
# plt.title('Horizontal Z Acceleration')
#
# plt.subplot(2,3,4)
# data = pandas.read_csv('10sec_vertical.csv')
# data['ax'].hist(bins=8)
# plt.title('Vertical X Acceleration')
# plt.subplot(2,3,5)
# data['ay'].hist(bins=8)
# plt.title('Vertical Y Acceleration')
# plt.subplot(2,3,6)
# data['az'].hist(bins=8)
# plt.title('Vertical Z Acceleration')
#

# graphs of position/velocity/acceleration
data1 = pandas.read_csv('./horizontal_to_vertical_1.csv')
data2 = pandas.read_csv('./horizontal_to_vertical_2.csv')
data3 = pandas.read_csv('./horizontal_to_vertical_3.csv')


def enc_or_pot_to_xy(data, column_name):
    xs = []
    zs = []
    thetas = []
    ARM_L = 280
    for i, row in enumerate(data.values):
        angle = row[column_name]
        x = ARM_L*np.sin(radians(angle))
        z = ARM_L*np.cos(radians(angle))
        xs.append(x)
        zs.append(z)
        theta = np.arctan2(x,z)
        thetas.append(theta)
    vthetas = np.diff(thetas, n=1)
    athetas = np.diff(thetas, n=2)

    return xs, zs, vthetas, athetas


def acc_to_xy(data):
    # mm/sec^2
    G = 9800
    # mm
    ARM_L = 280
    # sec
    dt = 0.01
    current_theta = 0
    current_z = 0
    current_x = ARM_L
    current_z_dot = 0
    current_x_dot = 0
    xs = []
    zs = []
    vs = []
    accs = []
    thetas = [0]
    for row in data.values:
        g_z = np.cos(current_theta) * G
        g_x = np.sin(current_theta) * G
        acc_x = row[2] * G - g_x
        acc_z = row[4] * G + g_z

        current_z_dot = acc_z * dt
        current_x_dot = acc_x * dt
        current_z += current_z_dot * dt + 1/2 * acc_z * dt * dt
        current_x += current_x_dot * dt + 1/2 * acc_x * dt * dt
        current_theta = np.arctan2(current_z, current_x)
        xs.append(-current_x)
        zs.append(current_z)
        thetas.append(current_theta)

    vs = np.diff(thetas, n=1)
    accs = np.diff(thetas, n=2)
    return xs, zs, -vs + 0.012, accs

pot_x1, pot_z1, pot_vel1, pot_acc1 = enc_or_pot_to_xy(data1, 0)
pot_x2, pot_z2, pot_vel2, pot_acc2 = enc_or_pot_to_xy(data2, 0)
pot_x3, pot_z3, pot_vel3, pot_acc3 = enc_or_pot_to_xy(data3, 0)


plt.figure(4)
plt.subplots_adjust(bottom=0.05, right=0.95, top=0.95, left=.05)
ax = plt.subplot(3, 3, 1)
ax.set_aspect('equal', 'datalim')
plt.title('Pot Cartesian Positions')
plt.plot(pot_x1, pot_z1, label='run 1')
plt.plot(pot_x2, pot_z2, label='run 2')
plt.plot(pot_x3, pot_z3, label='run 3')
plt.legend()

ax = plt.subplot(3, 3, 4)
plt.title('Pot Joint Velocity')
plt.plot(pot_vel1)
plt.plot(pot_vel2)
plt.plot(pot_vel3)

ax = plt.subplot(3, 3, 7)
plt.title('Pot Joint Acceleration')
plt.plot(pot_acc1)
plt.plot(pot_acc2)
plt.plot(pot_acc3)

enc_x1, enc_z1, enc_vel1, enc_acc1 = enc_or_pot_to_xy(data1, 1)
enc_x2, enc_z2, enc_vel2, enc_acc2 = enc_or_pot_to_xy(data2, 1)
enc_x3, enc_z3, enc_vel3, enc_acc3 = enc_or_pot_to_xy(data3, 1)

ax = plt.subplot(3, 3, 2)
ax.set_aspect('equal', 'datalim')
plt.title('Encoder Cartesian Positions')
plt.plot(enc_x1, enc_z1)
plt.plot(enc_x2, enc_z2)
plt.plot(enc_x3, enc_z3)

ax = plt.subplot(3, 3, 5)
plt.title('Encoder Joint Velocity')
plt.plot(enc_vel1)
plt.plot(enc_vel2)
plt.plot(enc_vel3)

ax = plt.subplot(3, 3, 8)
plt.title('Encoder Joint Acceleration')
plt.plot(enc_acc1)
plt.plot(enc_acc2)
plt.plot(enc_acc3)

acc_x1, acc_z1, acc_vel1, acc_acc1 = acc_to_xy(data1)
acc_x2, acc_z2, acc_vel2, acc_acc2 = acc_to_xy(data2)
acc_x3, acc_z3, acc_vel3, acc_acc3 = acc_to_xy(data3)

ax = plt.subplot(3, 3, 3)
ax.set_aspect('equal', 'datalim')
plt.title('Accelerometer Cartesian Positions')
plt.plot(acc_x1, acc_z1)
plt.plot(acc_x2, acc_z2)
plt.plot(acc_x3, acc_z3)

ax = plt.subplot(3, 3, 6)
plt.title('Accelerometer Joint Velocity')
plt.plot(acc_vel1)
plt.plot(acc_vel2)
plt.plot(acc_vel3)

ax = plt.subplot(3, 3, 9)
plt.title('Accelerometer Joint Acceleration')
plt.plot(acc_acc1)
plt.plot(acc_acc2)
plt.plot(acc_acc3)
plt.savefig('pos_vel_acc_all.png')




plt.figure(5)
plt.subplot(1, 3, 1)
plt.title("Position from each sensor source")
plt.plot(pot_x1, pot_z1)
plt.plot(enc_x1, enc_z1)
plt.plot(acc_x1, acc_z1)

plt.subplot(1, 3, 2)
plt.title("Velocity from each sensor source")
plt.plot(pot_vel1)
plt.plot(enc_vel1)
plt.plot(acc_vel1)

plt.subplot(1, 3, 3)
plt.title("Acceleration from each sensor source")
plt.plot(pot_acc1, label='pot')
plt.plot(enc_acc1, label='encoder')
plt.plot(acc_acc1, label='accelerometer')
plt.legend()
plt.savefig('pos_vel_acc_overlayed.png')

plt.show()
