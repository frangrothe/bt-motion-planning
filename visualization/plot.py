# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import sys

import numpy as np
import matplotlib.pyplot as plt
from pandas import read_csv



def plot_motion_plan(filename):
    df = load_csv(filename)
    x_index = 0
    t_index = 1
    in_edge_index = 2

    for i in range(len(df.index)):
        if df.iloc[i, in_edge_index] < 0:
            continue
        else:
            j = df.iloc[i, in_edge_index]
            plt.plot([df.iloc[i, x_index], df.iloc[j, x_index]],
                     [df.iloc[i, t_index], df.iloc[j, t_index]], '.-k')
    # plot solution path
    solution_length = 0
    current = 0
    while current >= 0:
        next = df.iloc[current, in_edge_index]
        if next >= 0:
            plt.plot([df.iloc[current, x_index], df.iloc[next, x_index]],
                     [df.iloc[current, t_index], df.iloc[next, t_index]], 'o-r', linewidth=2)
        current = next
        solution_length += 1

    print('\nSolution Length: ' + str(solution_length))
    plt.show()

def load_csv(filename):
    df = read_csv('data/' + filename + '.csv')
    return df


# Call with csv file to plot as command line argument
if __name__ == '__main__':
    filename = sys.argv[1]

    plot_motion_plan(filename)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
