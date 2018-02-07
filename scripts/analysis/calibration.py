#!/usr/bin/env python

import os
import matplotlib.pyplot as plt
import numpy as np

LOG_PATH = os.path.join(os.path.expanduser('~'), "Desktop")


def plot(filepath, num):
    angles = []

    with open(filepath, "r") as f:
        for line in f:
            angle = abs(float(line))
            angles.append(angle)

    plt.figure()
    plt.plot(range(len(angles)), angles, 'bx')

    filename = LOG_PATH + "/screenshots/angles_"
    i=1
    while os.path.exists('{}{:d}.png'.format(filename, i)):
        i += 1
    filename = '{}{:d}.png'.format(filename, i)

    plt.savefig(filename)

    print("mean is: {}".format(np.mean(angles)))
    print("median is: {}".format(np.median(angles)))
    print("variance is {}".format(np.var(angles)))
    print("std is {}".format(np.std(angles)))
