# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import sys

import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import axes3d
from matplotlib.patches import Rectangle, Patch
from matplotlib.lines import Line2D
from pandas import read_csv

# colorblind-friendly colors from the IBM Design Library
# https://davidmathlogic.com/colorblind/#%23648FFF-%23785EF0-%23DC267F-%23FE6100-%23FFB000
ibm_blue = '#648FFFaa'
ibm_violet = '#785EF0'
ibm_red = '#DC267F'
ibm_orange = '#FE6100'
ibm_yellow = '#FFB000'

# sample & path data indexes
x_index = 0
y_index = 1
t_index = 2
in_edge_index = 3

# goal indexes
t_low_index = 2
t_high_index = 3


def plot_motion_plan(filename):
    df_samples = read_csv('data/' + filename + '/samples.csv')
    df_path = read_csv('data/' + filename + '/path.csv')
    df_goal = read_csv('data/' + filename + '/goal.csv')
    # df_constraints = read_csv('data/' + filename + '/constraints.csv')
    # df_path = read_csv('data/' + filename + '/path.csv')
    # df_goal = read_csv('data/' + filename + '/goal.csv')

    # draw base graph
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # draw constraints
    polygons = plot_constraints(filename)
    for poly in polygons:
        ax.add_collection3d(poly)

    for i in range(len(df_samples.index)):
        in_edges = [int(x) for x in df_samples.iloc[i, in_edge_index].split('#') if x]
        for e in in_edges:
            ax.plot([df_samples.iloc[i, x_index], df_samples.iloc[e, x_index]],
                    [df_samples.iloc[i, y_index], df_samples.iloc[e, y_index]],
                    [df_samples.iloc[i, t_index], df_samples.iloc[e, t_index]], color='black', zorder=3000)
    ax.scatter(df_samples['x'], df_samples['y'], df_samples['time'], color='black', marker='o', zorder=5000, label='samples')

    # draw solution path
    ax.plot(df_path['x'].to_numpy(), df_path['y'].to_numpy(), df_path['time'].to_numpy(),
            'D-', linewidth=2, color=ibm_red, zorder=8000, label='solution')

    # draw Goal line
    x_goal = df_goal.iloc[0, x_index]
    y_goal = df_goal.iloc[0, y_index]
    t_low = df_goal.iloc[0, t_low_index]
    t_high = df_goal.iloc[0, t_high_index]
    ax.plot([x_goal, x_goal],
            [y_goal, y_goal],
            [t_low, t_high], color=ibm_yellow, zorder=6000, label='goal')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')

    # create handles for the legend
    sample_line = Line2D([], [], color='black', marker='.',
                              markersize=6, label='samples')
    path_line = Line2D([], [], color=ibm_red, marker='D',
                         markersize=6, label='solution path')
    constraint_patch = Patch(color=ibm_blue, label='constraints')
    goal_line = Line2D([], [], color=ibm_yellow, label='goal')

    ax.legend(handles=[sample_line, path_line, constraint_patch, goal_line])
    plt.show()


def plot_constraints(filename):
    with open('data/' + filename + '/constraints.json') as f:
        data = json.load(f)
    polygons = []
    for constraint in data:
        v_pre = [tuple(x) for x in constraint[0]]
        for i in range(1, len(constraint)):
            v_next = [tuple(x) for x in constraint[i]]
            for j in range(4):
                k = 0 if j == 3 else j + 1
                vertices = [v_pre[j], v_next[j], v_next[k], v_pre[k]]
                poly = Poly3DCollection([vertices], alpha=0.6, color=ibm_blue, zorder=0)
                polygons.append(poly)
            v_pre = v_next
    return polygons

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(df_samples['x'], df_samples['y'], df_samples['time'])
    # plt.show()

    #     in_edges = df_samples.iloc[i, in_edge_index].split('#')
    #     for e in in_edges:
    #         if int(e) < 0:
    #             continue
    #         else:
    #             j = int(e)
    #             plt.plot([df_samples.iloc[i, x_index], df_samples.iloc[j, x_index]],
    #                      [df_samples.iloc[i, t_index], df_samples.iloc[j, t_index]], '.-k')
    # # plot solution path
    # plt.plot(df_path['x'].to_numpy(), df_path['time'].to_numpy(), 'D-', linewidth=2, color=ibm_red)
    # print('\nSolution Path Length: ' + str(len(df_path.index)))
    #
    # # draw constraints
    # current_axis = plt.gca()
    # for i in range(len(df_constraints.index)):
    #     x_lb = df_constraints.iloc[i, x_lb_index]
    #     t_lb = df_constraints.iloc[i, t_lb_index]
    #     x_diff = df_constraints.iloc[i, x_ub_index] - df_constraints.iloc[i, x_lb_index]
    #     t_diff = df_constraints.iloc[i, t_ub_index] - df_constraints.iloc[i, t_lb_index]
    #     current_axis.add_patch(Rectangle((x_lb, t_lb), x_diff, t_diff, facecolor=ibm_blue))
    #
    # # draw goal region
    # x_diff = df_goal.iloc[0, x_ub_index] - df_goal.iloc[0, x_lb_index]
    # t_diff = df_goal.iloc[0, t_ub_index] - df_goal.iloc[0, t_lb_index]
    # current_axis.add_patch(Rectangle((df_goal.iloc[0, x_lb_index], df_goal.iloc[0, t_lb_index]), x_diff, t_diff, facecolor=ibm_yellow))
    #
    # # create handles for the legend
    # sample_line = Line2D([], [], color='black', marker='.',
    #                           markersize=6, label='samples')
    # path_line = Line2D([], [], color=ibm_red, marker='D',
    #                      markersize=6, label='solution path')
    # constraint_patch = Patch(color=ibm_blue, label='constraints')
    # goal_patch = Patch(color=ibm_yellow, label='goal region')
    #
    # plt.legend(handles=[sample_line, path_line, constraint_patch, goal_patch])
    # plt.xlabel("distance x")
    # plt.ylabel("time t")
    # plt.show()


# Call with csv file to plot as command line argument
if __name__ == '__main__':
    filename = sys.argv[1]
    plot_motion_plan(filename)

    # plot_motion_plan(filename)
    # ts = np.linspace(0.0, 1.0, 10)
    # xlow = 0.3
    # xhigh = 0.4
    # ylow = 0.6
    # yhigh = 0.7
    # v = 0.2
    #
    # def xfunc(start, t):
    #     a = 0.04
    #     v = 0.2
    #     return start + v * t + 0.5 * a * t**2
    #
    # def y_func(start, t):
    #     a = 0.15
    #     v = 0.01
    #     return start + v * t + 0.5 * a * t ** 2
    #
    #
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # polygons = []
    #
    # x_pre = [xfunc(xlow, ts[0]), xfunc(xlow, ts[0]), xfunc(xhigh, ts[0]), xfunc(xhigh, ts[0])]
    # y_pre = [y_func(yhigh, ts[0]), y_func(ylow, ts[0]), y_func(ylow, ts[0]), y_func(yhigh, ts[0])]
    # t_pre = [ts[0], ts[0], ts[0], ts[0]]
    # v_pre = list(zip(x_pre, y_pre, t_pre))
    #
    # for i in range(1, len(ts)):
    #     x_next = [xfunc(xlow, ts[i]), xfunc(xlow, ts[i]), xfunc(xhigh, ts[i]), xfunc(xhigh, ts[i])]
    #     y_next = [y_func(yhigh, ts[i]), y_func(ylow, ts[i]), y_func(ylow, ts[i]), y_func(yhigh, ts[i])]
    #     t_next = [ts[i], ts[i], ts[i], ts[i]]
    #     v_next = list(zip(x_next, y_next, t_next))
    #     print(v_pre)
    #     print(v_next)
    #     for j in range(4):
    #         k = 0 if j == 3 else j + 1
    #         vertices = [v_pre[j], v_next[j], v_next[k], v_pre[k]]
    #         poly = Poly3DCollection([vertices], alpha=0.8, color=ibm_blue)
    #         polygons.append(poly)
    #     v_pre = v_next
    #
    # for poly in polygons:
    #     ax.add_collection3d(poly)
    # plt.show()

    # x_edges = [0.3, 0.3, 0.5, 0.5]
    # y_edges = [0.7, 0.5, 0.5, 0.7]
    # t_edges = [0.0, 0.0, 0.0, 0.0]
    #
    # x2_edges = [0.4, 0.4, 0.6, 0.6]
    # y2_edges = [0.8, 0.6, 0.6, 0.8]
    # t2_edges = [0.5, 0.5, 0.5, 0.5]
    #
    # vertices = list(zip(x_edges, y_edges, t_edges))
    # print(vertices)
    # vertices2 = list(zip(x2_edges, y2_edges, t2_edges))
    # print(vertices2)
    # vertices3 = [vertices[0], vertices2[0], vertices2[1], vertices[1]]
    # vertices4 = [vertices[1], vertices2[1], vertices2[2], vertices[2]]
    # vertices5 = [vertices[3], vertices2[3], vertices2[0], vertices[0]]
    #
    # poly = Poly3DCollection([vertices], alpha=0.8)
    # poly2 = Poly3DCollection([vertices2], alpha=0.8)
    # poly3 = Poly3DCollection([vertices3], alpha=0.8)
    # poly4 = Poly3DCollection([vertices4], alpha=0.8)
    # poly5 = Poly3DCollection([vertices5], alpha=0.8)
    #
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.add_collection3d(poly)
    # ax.add_collection3d(poly2)
    # ax.add_collection3d(poly3)
    # ax.add_collection3d(poly4)
    # ax.add_collection3d(poly5)

    # Grab some test data.
    # X, Y, Z = axes3d.get_test_data(0.05)

    # Plot a basic wireframe.
    # ax.plot_surface(xs, ys, tts, rstride=1, cstride=1)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
