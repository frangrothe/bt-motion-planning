import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d

# agent_colors = ['red', 'purple', 'green', 'blue', 'orange', 'grey', 'yellow', 'pink']
agent_colors = ['#332288', '#117733', '#44AA99', '#88CCEE', '#DDCC77', '#CC6677', '#AA4499', '#882255']

t_max = 60

def plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    plot_constraints(ax)
    for i in range(8):
        plot_solution(ax, i)

    ax.set_xlim(0, 50)
    ax.set_ylim(0, 50)
    ax.set_zlim(0, 60)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')
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
        # Draw a circle on the x=0 'wall'
        p1 = Circle((x, y), r, color='black', alpha=0.6)
        p2 = Circle((x, y), r, color='black', alpha=0.6)
        ax.add_patch(p1)
        ax.add_patch(p2)
        art3d.pathpatch_2d_to_3d(p1, z=0, zdir="z")
        art3d.pathpatch_2d_to_3d(p2, z=t_max, zdir="z")
        # Draw lines between the circles
        num_lines = 32
        angles = np.linspace(0, 2 * np.pi, num_lines, endpoint=False)
        for angle in angles:
            xx = x + r * np.cos(angle)
            yy = y + r * np.sin(angle)

            ax.plot([xx, xx], [yy, yy], [0, t_max], color='black', alpha=0.6, linewidth=2)

def plot_solution(ax, index):
    with open('data/query/solution' + str(index) + '.json') as f:
        data = json.load(f)
    r = 1 # agent radius

    num_lines = 8
    angles = np.linspace(0, 2 * np.pi, num_lines, endpoint=False)
    next_i = 0
    for point in data:
        next_i += 1
        x = point[0]
        y = point[1]
        t = point[2]
        # Draw a circle on the x=0 'wall'
        p = Circle((x, y), r, color=agent_colors[index], alpha=0.6)
        ax.add_patch(p)
        art3d.pathpatch_2d_to_3d(p, z=t, zdir="z")
        # Draw lines between the circles
        if next_i < len(data):
            x_next = data[next_i][0]
            y_next = data[next_i][1]
            t_next = data[next_i][2]
            for angle in angles:
                x1 = x + r * np.cos(angle)
                y1 = y + r * np.sin(angle)
                x2 = x_next + r * np.cos(angle)
                y2 = y_next + r * np.sin(angle)
                ax.plot([x1, x2], [y1, y2], [t, t_next], color=agent_colors[index], alpha=0.6, linewidth=2)



if __name__ == '__main__':
    plot()