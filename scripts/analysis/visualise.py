import os
import matplotlib.pyplot as plt

LOG_PATH = os.path.join(os.path.expanduser('~'), "Desktop")


def plot(path_to_coordinates, i):
    xs = []
    ys = []

    with open(path_to_coordinates, "r") as mycoords:
        for line in mycoords:
            [x, y] = map(float, line.split(","))
            xs.append(x)
            ys.append(y)

    plt.figure()
    plt.plot(xs, ys, 'b:')
    traj_name = LOG_PATH + "/screenshots" + "/traj"
    plt.savefig('{}{:d}.png'.format(traj_name, i))


if __name__ == "__main__":
    # num = int(input("Enter log file number to visualise that trajectory:\n"))
    for i in range(600, 650):
        try:
            plot(LOG_PATH + "/logs/trajectory_log_{}.txt".format(i), i)
        except Exception as e:
            pass
