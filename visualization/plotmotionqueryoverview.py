# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import json

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

# colorblind-friendly colors from the IBM Design Library
# https://davidmathlogic.com/colorblind/#%23648FFF-%23785EF0-%23DC267F-%23FE6100-%23FFB000
# TOL color scheme
agent_colors = ['#332288', '#117733', '#44AA99', '#88CCEE', '#DDCC77', '#CC6677', '#AA4499', '#882255']



def plot_motion_plan():
    fig, ax = plt.subplots()
    plt.xlim(0, 50)
    plt.ylim(0, 50)
    plot_constraints(ax)
    for i in range(8):
        plot_discs(ax, i)
    plt.xlabel("x")
    plt.ylabel("y")

    # draw legend
    ob = Line2D([0], [0], marker='o', color='w', label='Obstacle',
                        markerfacecolor='black', markersize=15)
    handles = [ob]
    for i in range(8):
        l = 'Disc ' + str(i + 1)
        handle = Line2D([0], [0], marker='o', color='w', label=l,
                        markerfacecolor=agent_colors[i], markersize=15)
        handles.append(handle)
    plt.legend(handles=handles)
    plt.show()

def plot_constraints(ax):
    with open('data/query/constraints.json') as f:
        data = json.load(f)
    for constraint in data:
        x = constraint[0]
        y = constraint[1]
        r = constraint[2]
        # Draw a circle
        circle = Circle((x, y), r, color='black', alpha=1.0)
        ax.add_patch(circle)

def plot_discs(ax, index):
    with open('data/query/solution' + str(index) + '.json') as f:
        data = json.load(f)
    r = 1 # agent radius
    # draw start
    start_x = data[0][0]
    start_y = data[0][1]
    circle_start = Circle((start_x, start_y), r, color=agent_colors[index], alpha=1.0)
    ax.add_patch(circle_start)
    plt.text(start_x - 0.5, start_y - 3, str(index + 1))
    # draw goal
    goal_x = data[-1][0]
    goal_y = data[-1][1]
    circle_goal = Circle((goal_x, goal_y), r, color=agent_colors[index], alpha=1.0)
    ax.add_patch(circle_goal)
    plt.text(goal_x - 0.5, goal_y - 3, str(index + 1), color='red')

# Call with csv file to plot as command line argument
if __name__ == '__main__':
    plot_motion_plan()



# See PyCharm help at https://www.jetbrains.com/help/pycharm/
