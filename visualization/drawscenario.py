import json
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import numpy as np

# colorblind-friendly colors from the IBM Design Library
# https://davidmathlogic.com/colorblind/#%23648FFF-%23785EF0-%23DC267F-%23FE6100-%23FFB000
ibm_blue = '#648FFF'
ibm_violet = '#785EF0'
ibm_red = '#DC267F'
ibm_orange = '#FE6100'
ibm_yellow = '#FFB000'

start_color = ibm_orange
goal_color = ibm_yellow
obstacle_color = ibm_blue

def open_json(name):
    file = open("data/testsets/2/" + name, "r")
    json_string = file.read()
    data = json.loads(json_string)
    return data

def visualize():
    filename = 'a1.json'
    data = open_json(filename)

    plt.figure(figsize=(8,8))
    plt.style.use('test.mplstyle')
    plt.tick_params(
        axis='x',  # changes apply to the x-axis
        which='both',  # both major and minor ticks are affected
        bottom=False,  # ticks along the bottom edge are off
        top=False,  # ticks along the top edge are off
        labelbottom=False)  # labels along the bottom edge are off
    plt.tick_params(
        axis='y',  # changes apply to the x-axis
        which='both',  # both major and minor ticks are affected
        bottom=False,  # ticks along the bottom edge are off
        top=False,  # ticks along the top edge are off
        left=False,
        labelleft=False)  # labels along the bottom edge are off
    ax = plt.gca()
    ax.set(xlim=(0, 1), ylim=(0, 1))

    # draw start
    s = data["start"]
    agent = Circle((s[0], s[1]), data['radius'], color=start_color, alpha=1.0, zorder=2)
    ax.add_patch(agent)

    # draw goal
    g = data["goal"]
    goal = Circle((g[0], g[1]), data['radius'], color=goal_color, alpha=1.0,
                  zorder=0)
    ax.add_patch(goal)

    # draw obstacles
    obs = data["obstacles"]

    for o in obs:
        r = o["r"]
        start = o["path"][0]
        end = o["path"][4]
        c = Circle((start[0], start[1]), r, color=obstacle_color, alpha=1.0,
                  zorder=500)
        ax.add_patch(c)
        c = Circle((end[0], end[1]), r, color=obstacle_color, alpha=0.5,
                   zorder=100)
        ax.add_patch(c)
        path = np.array(o["path"])
        v = np.split(path, 2, axis=1)
        xs = v[0][:5]
        ys = v[1][:5]
        ax.plot(xs, ys, color=obstacle_color, ls='--', zorder=1)

    # plt.show()
    plt.savefig('random_scenario.pdf', format='pdf', dpi=300, bbox_inches='tight')


if __name__ == '__main__':
    visualize()