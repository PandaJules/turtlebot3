#!/usr/bin/env python

import os
import matplotlib.pyplot as plt
import numpy as np

LOG_PATH = os.path.join(os.path.expanduser('~'), "Desktop")


def plot(filepath):
    angles = []

    with open(filepath, "r") as f:
        for line in f:
            angle = float(line)
            angles.append(angle)

    plt.figure()
    plt.plot(range(len(angles)), angles, 'bx')
    plt.savefig(LOG_PATH + "/angle2.png")

    print("mean is: {}".format(np.mean(angles)))
    print("median is: {}".format(np.median(angles)))
    print("variance is {}".format(np.var(angles)))
    print("std is {}".format(np.std(angles)))


plot("/home/julia/Desktop/angles3.txt")
