#!/usr/bin/python3.5

import matplotlib.pyplot as plt
import numpy as np
import pandas

# histrograms of delta encoder counts
for i, ext in enumerate(['3', '6', '-3']):
    f = open('./encoder_ticks_%s.csv'%ext)

    differences = []
    last_datum = 0
    for j, line in enumerate(f.readlines()):
        datum = int(line.strip("\n"))
        if j > 0:
            diff = last_datum - int(datum)
            differences.append(diff)
        last_datum = int(datum)

    plt.figure(i)
    bins = list(np.arange(-11.5,11.5))
    plt.hist(differences, bins)
    plt.xlabel("Count difference")
    plt.xlim(-11.5, 11.5)
    plt.title("Change in Encoder Ticks at %s volts"%ext)
    plt.savefig('encoder_ticks_%s.png' % ext)

# histograms of accelerometer data
plt.subplot(2,3,1)
data = pandas.read_csv('10sec_horizontal.csv')
data['ax'].hist(bins=8)
plt.title('Horizontal X Acceleration')
plt.subplot(2,3,2)
data['ay'].hist(bins=8)
plt.title('Horizontal Y Acceleration')
plt.subplot(2,3,3)
data['az'].hist(bins=8)
plt.title('Horizontal Z Acceleration')

plt.subplot(2,3,4)
data = pandas.read_csv('10sec_vertical.csv')
data['ax'].hist(bins=8)
plt.title('Vertical X Acceleration')
plt.subplot(2,3,5)
data['ay'].hist(bins=8)
plt.title('Vertical Y Acceleration')
plt.subplot(2,3,6)
data['az'].hist(bins=8)
plt.title('Vertical Z Acceleration')

# graphs of position/velocity/acceleration
plt.show()

