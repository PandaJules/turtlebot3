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
    plt.plot(xs, ys, 'r-')

    plt.grid(True)
    plt.ylabel('Y coordinate', fontsize=20)
    plt.xlabel('X coordinate', fontsize=20)

    traj_name = LOG_PATH + "/screenshots" + "/traj"
    plt.savefig('{}{:d}.png'.format(traj_name, i))


if __name__ == "__main__":
    choice = "none"
    n = []
    while choice != 'r' and choice != 'n':
        choice = raw_input("Do you want to enter a range (r) or a number(n)?\t")
        if choice == 'r':
            a = int(input("Give me the first number of the range\n"))
            b = int(input("Give me the last number of the range\n"))
            n = range(a, b+1)
        elif choice == 'n':
            a = int(input("Give me the number\n"))
            n = [a]
        else:
            print("You should reply with either r or n")
    for i in n:
        try:
            plot(LOG_PATH + "/logs/trajectory_log_{}.txt".format(i), i)
        except Exception as e:
            print(e)
