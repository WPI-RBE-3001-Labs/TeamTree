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
    ('sin_125.csv', 125, 'sine'),
    ('sin_140.csv', 140, 'sine'),
    ('triag_20.csv', 20, 'triangle'),
    ('triag_125.csv', 125, 'triangle'),
    ('triag_140.csv', 140, 'triangle'),
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
    name = wave_type + '_' + str(freq) + 'hz'
    plt.plot(times, sampled_voltages)

    plt.title(name)
    plt.xlabel('time')
    plt.ylabel('voltage')

    # save
    plt.savefig(name + '.png')

    i += 1

# Plots for generated waveforms
pwms = [
    ('100hz_25duty.csv', 100, 25),
    ('100hz_70duty.csv', 100, 70),
    ('20hz_25duty.csv', 20, 25),
    ('20hz_70duty.csv', 20, 70),
    ('1hz_25duty.csv', 1, 25),
    ('1hz_70duty.csv', 1, 70),
    ]

for filename, freq, duty, in pwms:
    csv_file = open(filename, 'rb')
    data = csv.reader(csv_file)

    output_state = []
    pot_voltage = []
    for row in data:
        try:
            output_state.append(float(row[2]))
            pot_voltage.append(float(row[3]))
        except ValueError:
            pass

    plt.figure(i)
    name = str(freq) + 'freq_' + str(duty) + 'duty'
    plt.title(name)
    line1, = plt.plot(output_state, label='output state')
    line2, = plt.plot(pot_voltage, label='pot voltage')
    plt.ylim((0, 5))
    plt.xlabel('time')

    # Create a legend for the first line.
    first_legend = plt.legend(handles=[line1], loc=4)

    # Add the legend manually to the current Axes.
    ax = plt.gca().add_artist(first_legend)

    # Create another legend for the second line.
    plt.legend(handles=[line2], loc=1)

    # save
    plt.savefig(name + '.png')

    i += 1

# Comment this back in if you want to be spammed by all the plots
#plt.show()
