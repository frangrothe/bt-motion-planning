import json
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import numpy as np

start_color = 'red'
goal_color = 'black'
obstacle_color = 'blue'


def open_json():
    filename = '07-31 15:58:33.json'
    file = open("data/testsets/2/" + filename, "r")
    json_string = file.read()
    dict = json.loads(json_string)
    return dict

def visualize():
    data = open_json()
    for time in range(8):
        for frac in range(10):
            visualize_time(data, time, frac)




def visualize_time(data, time, frac):
    fig, ax = plt.subplots()
    plt.xlabel("x")
    plt.ylabel("y")
    ax.set_aspect('equal', 'box')
    ax.set(xlim=(0, 1), ylim=(0, 1))
    # plot start
    start = Circle((data['start'][0], data['start'][1]), data['radius'], color=start_color, alpha=1.0)
    ax.add_patch(start)

    # plot goal
    goal = Circle((data['goal'][0], data['goal'][1]), data['radius'], color=goal_color, alpha=1.0)
    ax.add_patch(goal)

    # plot obstacles
    if frac == 0:
        for o in data['obstacles']:
            c = Circle((o['path'][time][0], o['path'][time][1]), o['r'], color=obstacle_color, alpha=1.0)
            ax.add_patch(c)
    else:
        for o in data['obstacles']:
            i = time
            next_i = time + 1 if time != 7 else 0
            start = np.asarray(o['path'][i])
            direction = np.asarray(o['path'][next_i]) - start
            pos = start + (0.1 * frac * direction)
            c = Circle((pos[0], pos[1]), o['r'], color=obstacle_color, alpha=1.0)
            ax.add_patch(c)

    fig.savefig('data/testsets/2/images/' + str(time) + '_' + str(frac) + '.png')
    plt.close(fig)

if __name__ == '__main__':
    visualize()