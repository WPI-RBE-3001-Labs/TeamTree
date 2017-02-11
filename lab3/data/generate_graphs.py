#!/usr/bin/python3.5

import matplotlib.pyplot as plt
import numpy as np
import pandas

for i, ext in enumerate(['3', '6', '-3']):
    f = open('./encoder_ticks_%s.csv'%ext)

    differences = []
    last_datum = 0
    for j, line in enumerate(f.readlines()):
        datum = int(line.strip("\n"))
        if j > 0:
            diff = last_datum - int(datum)
            if i == 3:
                print(diff)
            differences.append(diff)
        last_datum = int(datum)

    plt.figure(i)
    print(i, np.amax(differences))
    bins = list(np.arange(-11.5,11.5))
    plt.hist(differences, bins)
    plt.xlabel("Count difference")
    plt.xlim(-11.5, 11.5)
    plt.title("Change in Encoder Ticks at %s volts"%ext)
    plt.savefig('encoder_ticks_%s.png' % ext)

plt.show()

