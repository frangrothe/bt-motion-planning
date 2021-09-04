import sqlite3
import numpy as np
import matplotlib.pyplot as plt

# colors
col1 = '#d7191c'
col2 = '#fdae61'
col3 = '#ffffbf'
col4 = '#abd9e9'
col5 = '#2c7bb6'

col_space_time_rrt = col5

# general parameters
resolution = 200
times = np.logspace(-2, 0.7, resolution)
eps = 0.002
max_cost = 10

def get_plot_data(cur):

    medians = np.zeros(len(times))
    percentile_low = np.zeros(len(times))
    percentile_high = np.zeros(len(times))

    for i in range(len(times)):
        t = times[i]
        data = np.array(cur.execute(
            "SELECT a.best_cost FROM (SELECT MAX(time), best_cost FROM {0} WHERE time<={1} GROUP BY runid) a".
            format('progress', times[i] + eps)).fetchall()).flatten()
        if data.size == 0:
            medians[i] = max_cost
            percentile_low[i] = max_cost
            percentile_high[i] = max_cost
        else:
            data = np.where(data == None, max_cost, data)
            medians[i] = np.median(data)
            percentile_low[i] = np.percentile(data, 5, interpolation='nearest')
            percentile_high[i] = np.percentile(data, 95, interpolation='nearest')

    return [medians, percentile_low, percentile_high]

def get_start_index(medians):
    for i in range(len(times)):
        if medians[i] < max_cost:
            return i
    return len(times)


def plot_optimization():
    con = sqlite3.connect('2/a1_spacetime.db')
    cur = con.cursor()
    # print(4 * '\n')
    # planners = cur.execute("SELECT id, name FROM {}".format('plannerConfigs')).fetchall()
    # for planner in planners:
    #     print("Planner {} has ID {}".format(planner[1], planner[0]))

    spacetime_rrt = get_plot_data(cur)

    plt.xscale('log')
    plt.xlim(0.01, 5.0)

    plt.yscale('linear')
    plt.ylim(0.0, max_cost)


    start = get_start_index(spacetime_rrt[0])
    plt.scatter(times[start:], spacetime_rrt[0][start:], color=col_space_time_rrt, label='SpaceTimeRRT')
    # Plot the confidence interval

    plt.fill_between(times[start:], spacetime_rrt[1][start:], spacetime_rrt[2][start:], color=col_space_time_rrt, alpha=0.5)
    plt.legend()
    plt.xlabel("run time [s]")
    plt.ylabel("solution cost")

    plt.show()
    # plt.savefig('optimization.png')


if __name__ == '__main__':
    plot_optimization()
