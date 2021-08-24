import json
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import numpy as np

start_color = 'red'
goal_color = 'black'
obstacle_color = 'blue'


def open_json(name):
    file = open("data/testsets/2/" + name, "r")
    json_string = file.read()
    data = json.loads(json_string)
    return data

def visualize():
    filename = 'a1.json'
    config_data = open_json(filename)
    solution = open_json('solutionRRTConnect_' + filename)

    finish_time = solution[-1]['time']
    for time in range(int(finish_time) + 1):
        for frac in range(10):
            visualize_time(config_data, solution, time, frac)
            print(f"{time}.{frac} finished.")




def visualize_time(config_data, solution, time, frac):
    fig, ax = plt.subplots()
    plt.xlabel("x")
    plt.ylabel("y")
    ax.set_aspect('equal', 'box')
    ax.set(xlim=(0, 1), ylim=(0, 1))

    # plot agent
    total_time = time + 0.1 * frac
    agent_pos = [0.0, 0.0]
    if total_time > solution[-1]['time']:
        agent_pos[0] = solution[-1]['pos'][0]
        agent_pos[1] = solution[-1]['pos'][1]
    else:
        index = 0
        for i in range(len(solution)):
            if solution[i]['time'] > total_time:
                index = i
                break
        start = np.asarray(solution[index - 1]['pos'])
        direction = np.asarray(solution[index]['pos']) - start
        scalar = (total_time - solution[index - 1]['time']) / (solution[index]['time'] - solution[index - 1]['time'])
        agent_pos = start + (scalar * direction)

    agent = Circle((agent_pos[0], agent_pos[1]), config_data['radius'], color=start_color, alpha=1.0, zorder=2)
    ax.add_patch(agent)

    # plot goal
    goal = Circle((config_data['goal'][0], config_data['goal'][1]), config_data['radius'], color=goal_color, alpha=1.0, zorder=0)
    ax.add_patch(goal)

    # plot obstacles
    if frac == 0:
        for o in config_data['obstacles']:
            c = Circle((o['path'][time][0], o['path'][time][1]), o['r'], color=obstacle_color, alpha=1.0)
            ax.add_patch(c)
    else:
        for o in config_data['obstacles']:
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