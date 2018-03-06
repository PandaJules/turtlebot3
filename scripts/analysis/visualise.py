from trajectory import plot
import os

LOG_PATH = os.path.join(os.path.expanduser('~'), "Desktop")

for i in [1,2,3,4, 5]:
    try:
        plot(LOG_PATH + "/logs/trajectory_log_{}.txt".format(i), i)
    except Exception as e:
        pass
