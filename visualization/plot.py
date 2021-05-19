# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from pandas import read_csv

# solution data indexes
x_index = 0
t_index = 1
in_edge_index = 2

# constraint indexes
c_x_lb_index = 0
c_x_ub_index = 1
c_t_lb_index = 2
c_t_ub_index = 3

def plot_motion_plan(filename):
    df = read_csv('data/' + filename + '.csv')
    constraints = read_csv('data/' + filename + '_constraints.csv')

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
        next_element = df.iloc[current, in_edge_index]
        if next_element >= 0:
            plt.plot([df.iloc[current, x_index], df.iloc[next_element, x_index]],
                     [df.iloc[current, t_index], df.iloc[next_element, t_index]], 'o-r', linewidth=2)
        current = next_element
        solution_length += 1

    print('\nSolution Length: ' + str(solution_length))

    # draw constraints
    current_axis = plt.gca()
    for i in range(len(constraints.index)):
        x_lb = constraints.iloc[i, c_x_lb_index]
        t_lb = constraints.iloc[i, c_t_lb_index]
        x_diff = constraints.iloc[i, c_x_ub_index] - constraints.iloc[i, c_x_lb_index]
        t_diff = constraints.iloc[i, c_t_ub_index] - constraints.iloc[i, c_t_lb_index]
        current_axis.add_patch(Rectangle((x_lb, t_lb), x_diff, t_diff, facecolor="blue"))

    plt.show()

def load_csv(filename):
    df = read_csv('data/' + filename + '.csv')
    print(df)
    return df


# Call with csv file to plot as command line argument
if __name__ == '__main__':
    filename = sys.argv[1]

    plot_motion_plan(filename)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
