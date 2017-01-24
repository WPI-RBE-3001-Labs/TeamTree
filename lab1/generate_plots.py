#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy as np
import csv

# Plots for generated waveforms
waveforms = [
    ('square_20.csv', 20, 'square'),
    ('square_125.csv', 125, 'square'),
    ('square_140.csv', 140, 'square'),
    ('sin_20.csv', 20, 'sin'),
    ('sin_125.csv', 125, 'sin'),
    ('sin_140.csv', 140, 'sin'),
    ('triag_20.csv', 20, 'triag'),
    ('triag_125.csv', 125, 'triag'),
    ('triag_140.csv', 140, 'triag'),
    ]

i = 0
for filename, freq, wave_type in waveforms:
    csv_file = open(filename, 'rb')
    data = csv.reader(csv_file)

    times = []
    sampled_voltages = []
    for row in data:
        try:
            times.append(float(row[0]))
            sampled_voltages.append(float(row[1]))
        except ValueError:
            pass

    plt.figure(i)
    plt.title(wave_type + '_' + str(freq))
    plt.plot(times, sampled_voltages)

    i += 1

plt.show()
