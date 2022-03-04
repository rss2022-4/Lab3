import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    with open("new_one_meter.txt") as f1:
        one_lines = f1.readlines()
    one_lines = np.array([float(line[6:-1]) for line in one_lines if 'data' in line])
    with open("new_half_meter.txt") as f2:
        half_lines = f2.readlines()
    half_lines = np.array([float(line[6:-1]) for line in half_lines if 'data' in line])
    half_x = np.linspace(0, 100, len(half_lines))
    one_x = np.linspace(0, 100, len(one_lines))

    plt.plot(half_x, half_lines)
    plt.plot(one_x, one_lines)
    plt.xlabel('Relative Location')
    plt.ylabel('Error [m]')
    plt.legend(['0.5 m/s', '1.0 m/s'])
    plt.show()

