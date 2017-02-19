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


ax = plt.subplot(3, 3, 1)
ax.set_aspect('equal', 'datalim')
plt.subplot(3, 3, 1)
plt.title('Pot Angles')
ARM_L = 280
xy1 = data1['encoder angle'].apply(lambda x: pandas.Series({'x': ARM_L*np.sin(radians(x)), 'y': ARM_L*np.cos(radians(x))}))
xy2 = data2['encoder angle'].apply(lambda x: pandas.Series({'x': ARM_L*np.sin(radians(x)), 'y': ARM_L*np.cos(radians(x))}))
xy3 = data3['encoder angle'].apply(lambda x: pandas.Series({'x': ARM_L*np.sin(radians(x)), 'y': ARM_L*np.cos(radians(x))}))
xy2.plot(x='x', y='y', ax=ax)
xy1.plot(x='x', y='y', ax=ax)
xy3.plot(x='x', y='y', ax=ax)

ax = plt.subplot(3, 3, 2)
ax.set_aspect('equal', 'datalim')
plt.title('Encoder Angles')
xy1 = data1['encoder angle'].apply(lambda x: pandas.Series({'x': ARM_L*np.sin(radians(x)), 'y': ARM_L*np.cos(radians(x))}))
xy2 = data2['encoder angle'].apply(lambda x: pandas.Series({'x': ARM_L*np.sin(radians(x)), 'y': ARM_L*np.cos(radians(x))}))
xy3 = data3['encoder angle'].apply(lambda x: pandas.Series({'x': ARM_L*np.sin(radians(x)), 'y': ARM_L*np.cos(radians(x))}))
xy2.plot(x='x', y='y', ax=ax)
xy1.plot(x='x', y='y', ax=ax)
xy3.plot(x='x', y='y', ax=ax)

ax = plt.subplot(3, 3, 3)
ax.set_aspect('equal', 'datalim')
plt.title('Accelerometer Angles')

def acc_to_xy(data):
    thetas = []
    current_theta = 0
    current_w = 0
    current_z = 0
    current_x = 0
    # mm/s^2
    G = 9.8 * 1600
    acc_z = G
    acc_x = 0
    dt = 0.01
    xs = []
    zs = []
    for row in data.values:
        g_z = np.cos(current_theta) * acc_z
        g_x = np.sin(current_theta) * acc_z
        acc_x = row[2] * G
        acc_z = row[4] * G

        current_z_dot = (acc_z - g_z) * dt
        current_x_dot = (acc_x - g_x) * dt
        current_z += current_z_dot * dt + 1/2 * acc_z * dt * dt
        current_x += current_x_dot * dt + 1/2 * acc_x * dt * dt
        current_theta = np.arctan2(current_z, current_x)
        thetas.append(current_theta)
        xs.append(-current_x)
        zs.append(current_z)

    return xs, zs

plt.plot(*acc_to_xy(data1))
plt.plot(*acc_to_xy(data2))
plt.plot(*acc_to_xy(data3))
plt.show()
