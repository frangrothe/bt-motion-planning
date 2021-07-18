# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Patch
from matplotlib.lines import Line2D
from pandas import read_csv

# colorblind-friendly colors from the IBM Design Library
# https://davidmathlogic.com/colorblind/#%23648FFF-%23785EF0-%23DC267F-%23FE6100-%23FFB000
ibm_blue = '#648FFF'
ibm_violet = '#785EF0'
ibm_red = '#DC267F'
ibm_orange = '#FE6100'
ibm_yellow = '#FFB000'

# sample & path data indexes
x_index = 0
t_index = 1
in_edge_index = 2

# constraint & goal region indexes
x_lb_index = 0
x_ub_index = 1
t_lb_index = 2
t_ub_index = 3

def plot_motion_plan(filename):
    df_samples = read_csv('data/' + filename + '/samples.csv')
    df_constraints = read_csv('data/' + filename + '/constraints.csv')
    df_path = read_csv('data/' + filename + '/path.csv')
    df_goal = read_csv('data/' + filename + '/goal.csv')

    # draw base graph
    for i in range(len(df_samples.index)):
        in_edges = [int(x) for x in df_samples.iloc[i, in_edge_index].split('#') if x]
        for e in in_edges:
            plt.plot([df_samples.iloc[i, x_index], df_samples.iloc[e, x_index]],
                     [df_samples.iloc[i, t_index], df_samples.iloc[e, t_index]], '.-k')
    # plot solution path
    plt.plot(df_path['x'].to_numpy(), df_path['time'].to_numpy(), 'D-', linewidth=2, color=ibm_red)
    print('\nSolution Path Length: ' + str(len(df_path.index)))

    # draw constraints
    current_axis = plt.gca()
    for i in range(len(df_constraints.index)):
        x_lb = df_constraints.iloc[i, x_lb_index]
        t_lb = df_constraints.iloc[i, t_lb_index]
        x_diff = df_constraints.iloc[i, x_ub_index] - df_constraints.iloc[i, x_lb_index]
        t_diff = df_constraints.iloc[i, t_ub_index] - df_constraints.iloc[i, t_lb_index]
        current_axis.add_patch(Rectangle((x_lb, t_lb), x_diff, t_diff, facecolor=ibm_blue))

    # draw goal region
    for i in range(len(df_goal.index)):
        x_diff = df_goal.iloc[i, x_ub_index] - df_goal.iloc[i, x_lb_index]
        t_diff = df_goal.iloc[i, t_ub_index] - df_goal.iloc[i, t_lb_index]
        if x_diff == 0:
            x_diff = 0.0001
        current_axis.add_patch(
            Rectangle((df_goal.iloc[i, x_lb_index], df_goal.iloc[i, t_lb_index]), x_diff, t_diff, facecolor=ibm_yellow))


    # create handles for the legend
    sample_line = Line2D([], [], color='black', marker='.',
                              markersize=6, label='samples')
    path_line = Line2D([], [], color=ibm_red, marker='D',
                         markersize=6, label='solution path')
    constraint_patch = Patch(color=ibm_blue, label='constraints')
    goal_patch = Patch(color=ibm_yellow, label='goal region')

    plt.legend(handles=[sample_line, path_line, constraint_patch, goal_patch])
    plt.xlabel("distance x")
    plt.ylabel("time t")
    plt.show()

def load_csv(filename):
    df = read_csv('data/' + filename + '.csv')
    print(df)
    return df


# Call with csv file to plot as command line argument
if __name__ == '__main__':
    filename = sys.argv[1]

    plot_motion_plan(filename)

    # s = "#3#4#5"
    # s = [x for x in s.split('#') if x]
    # print(s)


# See PyCharm help at https://www.jetbrains.com/help/pycharm/
