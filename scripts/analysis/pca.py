from trajectory import plot
import os

LOG_PATH = os.path.join(os.path.expanduser('~'), "Desktop")

for i in [46, 54, 56, 63, 64, 72, 82]:
    plot(LOG_PATH + "/logs/trajectory_log_{}.txt".format(i), i)