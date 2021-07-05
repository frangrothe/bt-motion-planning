

import sys
from pandas import read_csv
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle, Patch

# colorblind-friendly colors from the IBM Design Library
# https://davidmathlogic.com/colorblind/#%23648FFF-%23785EF0-%23DC267F-%23FE6100-%23FFB000
ibm_blue = '#648FFFaa'
ibm_violet = '#785EF0'
ibm_red = '#DC267F'
ibm_orange = '#FE6100'
ibm_yellow = '#FFB000'
black = '#000000'

# sample & path data indexes
x_index = 0
t_index = 1
in_edge_index = 2
start_or_goal_index = 3

# constraint & goal region indexes
x_lb_index = 0
x_ub_index = 1
t_lb_index = 2
t_ub_index = 3

def plot_motion_plan(fileindex):
    df_samples_pre = read_csv('data/debug/' + fileindex + 'prePruning.csv')
    df_samples_after = read_csv('data/debug/' + fileindex + 'afterPruning.csv')
    df_constraints = read_csv('data/testing/constraints.csv')
    df_goal = read_csv('data/testing/goal.csv')
    df_path = read_csv('data/debug/' + fileindex + 'path.csv')
    timebound = df_path.iloc[0, 2]

    fig, axs = plt.subplots(1, 2, constrained_layout=True)

    # draw graph before Pruning
    plot_single_graph(axs[0], df_samples_pre, df_constraints, df_goal, timebound, df_path)
    axs[0].set_title('Before Pruning')


    # draw graph after Pruning
    plot_single_graph(axs[1], df_samples_after, df_constraints, df_goal, timebound)
    axs[1].set_title('After Pruning')

    fig.suptitle('Solution ' + fileindex, fontsize=16)
    plt.show()

def plot_single_graph(ax, samples, constraints, goal, timebound, path=None):
    # draw constraints
    for i in range(len(constraints.index)):
        x_lb = constraints.iloc[i, x_lb_index]
        t_lb = constraints.iloc[i, t_lb_index]
        x_diff = constraints.iloc[i, x_ub_index] - constraints.iloc[i, x_lb_index]
        t_diff = constraints.iloc[i, t_ub_index] - constraints.iloc[i, t_lb_index]
        ax.add_patch(Rectangle((x_lb, t_lb), x_diff, t_diff, facecolor=ibm_orange))

    # draw goal region
    x_diff = goal.iloc[0, x_ub_index] - goal.iloc[0, x_lb_index]
    t_diff = goal.iloc[0, t_ub_index] - goal.iloc[0, t_lb_index]
    ax.add_patch(
        Rectangle((goal.iloc[0, x_lb_index], goal.iloc[0, t_lb_index]), x_diff, t_diff, facecolor=ibm_yellow, zorder=1))

    # draw start and goal tree
    for i in range(len(samples.index)):
        in_edges = [int(x) for x in samples.iloc[i, in_edge_index].split('#') if x]
        color = black if samples.iloc[i, start_or_goal_index] == 1 else ibm_blue
        for e in in_edges:
            ax.plot([samples.iloc[i, x_index], samples.iloc[e, x_index]],
                     [samples.iloc[i, t_index], samples.iloc[e, t_index]], '.-k', color=color)
        if len(in_edges) == 0:
            ax.scatter(samples.iloc[i, x_index], samples.iloc[i, t_index], marker='.', color=color, zorder=2)

    # draw solution path
    if path is not None:
        ax.plot(path['x'].to_numpy(), path['time'].to_numpy(), 'D-', linewidth=2, color=ibm_red)

    # draw cut off line
    ax.plot([0, 1], [timebound - 1, timebound], '--', color=ibm_violet)

    # create handles for the legend
    start_line = Line2D([], [], color=black, marker='.',
                              markersize=6, label='start tree')
    goal_line = Line2D([], [], color=ibm_blue, marker='.',
                        markersize=6, label='goal tree')
    constraint_patch = Patch(color=ibm_orange, label='constraints')
    goal_patch = Patch(color=ibm_yellow, label='goal region')
    cutoff_line = Line2D([], [], color=ibm_violet, linestyle='--',
                       markersize=6, label='time cutoff')
    path_line = Line2D([], [], color=ibm_red, marker='D',
                              markersize=6, label='solution path')

    if path is not None:
        ax.legend(handles=[start_line, goal_line, constraint_patch, goal_patch, cutoff_line, path_line])
    else:
        ax.legend(handles=[start_line, goal_line, constraint_patch, cutoff_line, goal_patch])
    ax.set_xlabel("distance x")
    ax.set_ylabel("time t")

if __name__ == '__main__':
    fileindex = sys.argv[1]

    plot_motion_plan(fileindex)