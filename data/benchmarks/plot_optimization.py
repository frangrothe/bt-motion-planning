import json
import sqlite3
import numpy as np
import matplotlib.pyplot as plt

# colors
col1 = '#d7191c'
col2 = '#fdae61'
col3 = '#ffffbf'
col4 = '#abd9e9'
col5 = '#2c7bb6'

col_space_time_rrt = col1
col_rrt_connect = col2
col_rrt_star = col4

filepath = '8/b1'
min_time = 0.05
max_time = 30

# general parameters
resolution = 200
times = np.logspace(np.log10(min_time), np.log10(max_time), resolution)
max_cost = 20

def get_plot_data(cur):

    medians = np.zeros(len(times))
    quantile5 = np.zeros(len(times))
    quantile95 = np.zeros(len(times))

    for i in range(len(times)):
        data = np.array(cur.execute(
            "SELECT a.best_cost FROM (SELECT MAX(time), best_cost FROM {0} WHERE time<={1} GROUP BY runid) a".
            format('progress', times[i])).fetchall()).flatten()
        if data.size == 0:
            medians[i] = max_cost
            quantile5[i] = max_cost
            quantile95[i] = max_cost
        else:
            data = np.where(data == None, max_cost, data)
            medians[i] = np.median(data)
            quantile5[i] = np.percentile(data, 5, interpolation='nearest')
            quantile95[i] = np.percentile(data, 95, interpolation='nearest')

    return [medians, quantile5, quantile95]

def get_start_index(medians):
    for i in range(len(times)):
        if medians[i] < max_cost:
            return i
    return len(times)


def store_data():
    con = sqlite3.connect(filepath + '_spacetime.db')
    cur = con.cursor()
    spacetime_rrt = get_plot_data(cur)

    con = sqlite3.connect(filepath + '_8.db')
    cur = con.cursor()
    rrt_star8 = get_plot_data(cur)

    con = sqlite3.connect(filepath + '_32.db')
    cur = con.cursor()
    rrt_star32 = get_plot_data(cur)

    con = sqlite3.connect(filepath + '_128.db')
    cur = con.cursor()
    rrt_star128 = get_plot_data(cur)

    with open(filepath + '.json', 'r') as jsonfile:
        data = json.load(jsonfile)

    data["spacetime"]["median"] = spacetime_rrt[0].tolist()
    data["spacetime"]["quantile5"] = spacetime_rrt[1].tolist()
    data["spacetime"]["quantile95"] = spacetime_rrt[2].tolist()
    data["rrtstar8"]["median"] = rrt_star8[0].tolist()
    data["rrtstar8"]["quantile5"] = rrt_star8[1].tolist()
    data["rrtstar8"]["quantile95"] = rrt_star8[2].tolist()
    data["rrtstar32"]["median"] = rrt_star32[0].tolist()
    data["rrtstar32"]["quantile5"] = rrt_star32[1].tolist()
    data["rrtstar32"]["quantile95"] = rrt_star32[2].tolist()
    data["rrtstar128"]["median"] = rrt_star128[0].tolist()
    data["rrtstar128"]["quantile5"] = rrt_star128[1].tolist()
    data["rrtstar128"]["quantile95"] = rrt_star128[2].tolist()

    with open(filepath + '.json','w') as jsonfile:
        json.dump(data, jsonfile, indent=4)


def plot():
    with open(filepath + '.json', 'r') as jsonfile:
        data = json.load(jsonfile)

    plt.xscale('log')
    plt.xlim(min_time, max_time)

    plt.yscale('linear')
    plt.ylim(0.0, max_cost)

    spacetime_median = data["spacetime"]["median"]
    spacetime_q5 = data["spacetime"]["quantile5"]
    spacetime_q95 = data["spacetime"]["quantile95"]
    start = get_start_index(spacetime_median)
    plt.plot(times[start:], spacetime_median[start:], color=col_space_time_rrt, label='SpaceTimeRRT')
    plt.fill_between(times[start:], spacetime_q5[start:], spacetime_q95[start:], color=col_space_time_rrt,
                     alpha=0.5)

    rrtstar8_median = data["rrtstar8"]["median"]
    rrtstar8_q5 = data["rrtstar8"]["quantile5"]
    rrtstar8_q95 = data["rrtstar8"]["quantile95"]

    start = get_start_index(rrtstar8_median)
    plt.plot(times[start:], rrtstar8_median[start:], color=col_rrt_star, label='RRTStar 8')
    plt.fill_between(times[start:], rrtstar8_q5[start:], rrtstar8_q95[start:], color=col_rrt_star,
                     alpha=0.5)

    rrtstar32_median = data["rrtstar32"]["median"]
    rrtstar32_q5 = data["rrtstar32"]["quantile5"]
    rrtstar32_q95 = data["rrtstar32"]["quantile95"]

    start = get_start_index(rrtstar32_median)
    plt.plot(times[start:], rrtstar32_median[start:], color=col_rrt_star, linestyle='--', label='RRTStar 32')
    plt.fill_between(times[start:], rrtstar32_q5[start:], rrtstar32_q95[start:], color=col_rrt_star,
                     alpha=0.5)

    rrtstar128_median = data["rrtstar128"]["median"]
    rrtstar128_q5 = data["rrtstar128"]["quantile5"]
    rrtstar128_q95 = data["rrtstar128"]["quantile95"]

    start = get_start_index(rrtstar128_median)
    plt.plot(times[start:], rrtstar128_median[start:], color=col_rrt_star, linestyle='dotted', label='RRTStar 128')
    plt.fill_between(times[start:], rrtstar128_q5[start:], rrtstar128_q95[start:], color=col_rrt_star,
                     alpha=0.5)

    # plot points with errorbars
    spacetime_point = data["spacetime"]["point"]
    time_errors, cost_errors = get_errors(spacetime_point)
    plt.errorbar(spacetime_point["time"][0], spacetime_point["cost"][0], cost_errors, time_errors, fmt='gx', ecolor='g', label='SpaceTimeRRT')

    rrtconnect8_point = data["rrtconnect8"]["point"]
    time_errors, cost_errors = get_errors(rrtconnect8_point)
    plt.errorbar(rrtconnect8_point["time"][0], rrtconnect8_point["cost"][0], cost_errors, time_errors, fmt='ys',
                 ecolor='y', label='RRTConnect8')

    rrtconnect32_point = data["rrtconnect32"]["point"]
    time_errors, cost_errors = get_errors(rrtconnect32_point)
    plt.errorbar(rrtconnect32_point["time"][0], rrtconnect32_point["cost"][0], cost_errors, time_errors, fmt='mo',
                 ecolor='m', label='RRTConnect32')

    # rrtconnect128_point = data["rrtconnect128"]["point"]
    # time_errors, cost_errors = get_errors(rrtconnect128_point)
    # plt.errorbar(rrtconnect128_point["time"][0], rrtconnect128_point["cost"][0], cost_errors, time_errors, fmt='bx',
    #              ecolor='black', label='RRTConnect128', zorder=1000)

    # rrtstar8_point = data["rrtstar8"]["point"]
    # time_errors, cost_errors = get_errors(rrtstar8_point)
    # plt.errorbar(rrtstar8_point["time"][0], rrtstar8_point["cost"][0], cost_errors, time_errors, fmt='m+',
    #              ecolor='black', label='RRTStar')


    plt.legend()
    plt.xlabel("run time [s]")
    plt.ylabel("solution cost")

    # plt.show()
    plt.savefig('dim8_optimization.png')

def get_errors(point):
    time_errors = np.array([[point["time"][0] - point["time"][1]],
                            [point["time"][2] - point["time"][0]]])
    cost_errors = np.array([[point["cost"][0] - point["cost"][1]],
                            [point["cost"][2] - point["cost"][0]]])
    return time_errors, cost_errors


if __name__ == '__main__':
    # store_data()
    plot()