import json
import sqlite3
import numpy as np
import matplotlib.pyplot as plt

# colors
ibm_blue = '#648FFF'
ibm_violet = '#785EF0'
ibm_red = '#DC267F'
ibm_orange = '#FE6100'
ibm_yellow = '#FFB000'

col_space_time_rrt = ibm_blue
col_rrt_connect = ibm_red
col_rrt_star = ibm_orange

filepath = 'narrow/n1'

# general parameters
resolution = 200


def get_plot_data(cur, times, max_cost, ci_left, ci_right):

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
            quantile5[i] = np.percentile(data, ci_left, interpolation='nearest')
            quantile95[i] = np.percentile(data, ci_right, interpolation='nearest')

    return [medians, quantile5, quantile95]

def get_start_index(medians, times, max_cost):
    for i in range(len(times)):
        if medians[i] < max_cost:
            return i
    return len(times)


def store_data():
    with open(filepath + '.json', 'r') as jsonfile:
        data = json.load(jsonfile)

    times = np.logspace(np.log10(data["info"]["min_time"]["optimization"]), np.log10(data["info"]["max_time"]["optimization"]), resolution)
    max_cost = data["info"]["max_cost"]
    ci_left = data["info"]["ci_left"]
    ci_right = data["info"]["ci_right"]

    con = sqlite3.connect(filepath + '_spacetime.db')
    cur = con.cursor()
    spacetime_rrt = get_plot_data(cur, times, max_cost, ci_left, ci_right)
    data["spacetime"]["median"] = spacetime_rrt[0].tolist()
    data["spacetime"]["quantile5"] = spacetime_rrt[1].tolist()
    data["spacetime"]["quantile95"] = spacetime_rrt[2].tolist()

    rrtstar_tb = data["info"]["rrtstar_tb"]
    for i in range(len(rrtstar_tb)):
        con = sqlite3.connect(filepath + '_' + str(rrtstar_tb[i]) + '.db')
        cur = con.cursor()
        rrt_star = get_plot_data(cur, times, max_cost, ci_left, ci_right)
        data["rrtstar" + str(rrtstar_tb[i])]["median"] = rrt_star[0].tolist()
        data["rrtstar" + str(rrtstar_tb[i])]["quantile5"] = rrt_star[1].tolist()
        data["rrtstar" + str(rrtstar_tb[i])]["quantile95"] = rrt_star[2].tolist()

    with open(filepath + '.json','w') as jsonfile:
        json.dump(data, jsonfile, indent=4)


def plot():
    with open(filepath + '.json', 'r') as jsonfile:
        data = json.load(jsonfile)

    min_time = data["info"]["min_time"]["optimization"]
    max_time = data["info"]["max_time"]["optimization"]
    times = np.logspace(np.log10(min_time), np.log10(max_time), resolution)
    max_cost = data["info"]["max_cost"]

    plt.style.use('../../test.mplstyle')
    plt.xscale('log')
    plt.xlim(min_time, max_time)

    plt.yscale('linear')
    plt.ylim(0.0, max_cost)

    spacetime_median = data["spacetime"]["median"]
    spacetime_q5 = data["spacetime"]["quantile5"]
    spacetime_q95 = data["spacetime"]["quantile95"]
    start = get_start_index(spacetime_median, times, max_cost)
    plt.plot(times[start:], spacetime_median[start:], color=col_space_time_rrt, label='ST-RRT*')
    plt.fill_between(times[start:], spacetime_q5[start:], spacetime_q95[start:], color=col_space_time_rrt,
                     alpha=0.5)

    linestyles = ['-', '--', 'dotted']
    markers = ['x', 'd', 'v']
    rrtstar_tb = data["info"]["rrtstar_tb"]
    for i in range(len(rrtstar_tb)):
        rrtstar_median = data["rrtstar" + str(rrtstar_tb[i])]["median"]
        rrtstar_q5 = data["rrtstar" + str(rrtstar_tb[i])]["quantile5"]
        rrtstar_q95 = data["rrtstar" + str(rrtstar_tb[i])]["quantile95"]
        start = get_start_index(rrtstar_median, times, max_cost)
        plt.plot(times[start:], rrtstar_median[start:], color=col_rrt_star, label='RRT* ' + str(rrtstar_tb[i]), linestyle=linestyles[i])
        plt.fill_between(times[start:], rrtstar_q5[start:], rrtstar_q95[start:], color=col_rrt_star,
                         alpha=0.5)

    rrtconnect_tb = data["info"]["rrtconnect_tb"]
    for i in range(len(rrtconnect_tb)):
        rrtconnect_point = data["rrtconnect" + str(rrtconnect_tb[i])]["point"]
        time_errors, cost_errors = get_errors(rrtconnect_point)
        plt.errorbar(rrtconnect_point["time"][0], rrtconnect_point["cost"][0], cost_errors, time_errors,
                     c=col_rrt_connect,
                     label='RRTConnect ' + str(rrtconnect_tb[i]), marker=markers[i], ms='8')



    # rrtstar8_median = data["rrtstar8"]["median"]
    # rrtstar8_q5 = data["rrtstar8"]["quantile5"]
    # rrtstar8_q95 = data["rrtstar8"]["quantile95"]
    #
    # start = get_start_index(rrtstar8_median)
    # plt.plot(times[start:], rrtstar8_median[start:], color=col_rrt_star, label='RRTStar 8')
    # plt.fill_between(times[start:], rrtstar8_q5[start:], rrtstar8_q95[start:], color=col_rrt_star,
    #                  alpha=0.5)

    # rrtstar32_median = data["rrtstar32"]["median"]
    # rrtstar32_q5 = data["rrtstar32"]["quantile5"]
    # rrtstar32_q95 = data["rrtstar32"]["quantile95"]
    #
    # start = get_start_index(rrtstar32_median)
    # plt.plot(times[start:], rrtstar32_median[start:], color=col_rrt_star, linestyle='--', label='RRTStar 32')
    # plt.fill_between(times[start:], rrtstar32_q5[start:], rrtstar32_q95[start:], color=col_rrt_star,
    #                  alpha=0.5)
    #
    # rrtstar128_median = data["rrtstar128"]["median"]
    # rrtstar128_q5 = data["rrtstar128"]["quantile5"]
    # rrtstar128_q95 = data["rrtstar128"]["quantile95"]

    # start = get_start_index(rrtstar128_median)
    # plt.plot(times[start:], rrtstar128_median[start:], color=col_rrt_star, linestyle='dotted', label='RRTStar 128')
    # plt.fill_between(times[start:], rrtstar128_q5[start:], rrtstar128_q95[start:], color=col_rrt_star,
    #                  alpha=0.5)

    # plot points with errorbars
    # spacetime_point = data["spacetime"]["point"]
    # time_errors, cost_errors = get_errors(spacetime_point)
    # plt.errorbar(spacetime_point["time"][0], spacetime_point["cost"][0], cost_errors, time_errors, c=col_space_time_rrt, label='SpaceTimeRRT',
    #              marker='v')
    #
    # rrtconnect8_point = data["rrtconnect8"]["point"]
    # time_errors, cost_errors = get_errors(rrtconnect8_point)
    # plt.errorbar(rrtconnect8_point["time"][0], rrtconnect8_point["cost"][0], cost_errors, time_errors, c=col_rrt_connect,
    #              label='RRTConnect8', marker='x')
    #
    # rrtconnect32_point = data["rrtconnect32"]["point"]
    # time_errors, cost_errors = get_errors(rrtconnect32_point)
    # plt.errorbar(rrtconnect32_point["time"][0], rrtconnect32_point["cost"][0], cost_errors, time_errors, c=col_rrt_connect,
    #              marker='d', label='RRTConnect32')

    # rrtconnect128_point = data["rrtconnect128"]["point"]
    # time_errors, cost_errors = get_errors(rrtconnect128_point)
    # plt.errorbar(rrtconnect128_point["time"][0], rrtconnect128_point["cost"][0], cost_errors, time_errors, fmt='bx',
    #              ecolor='black', label='RRTConnect128', zorder=1000)

    # rrtstar8_point = data["rrtstar8"]["point"]
    # time_errors, cost_errors = get_errors(rrtstar8_point)
    # plt.errorbar(rrtstar8_point["time"][0], rrtstar8_point["cost"][0], cost_errors, time_errors, fmt='m+',
    #              ecolor='black', label='RRTStar')


    plt.legend(loc='upper right')
    plt.grid(True, which="both", ls='--')
    plt.xlabel("run time [s]")
    plt.ylabel("solution cost")

    # plt.show()

    plt.savefig('narrow_optimization.pdf', format='pdf', dpi=300, bbox_inches='tight')

def get_errors(point):
    time_errors = np.array([[point["time"][0] - point["time"][1]],
                            [point["time"][2] - point["time"][0]]])
    cost_errors = np.array([[point["cost"][0] - point["cost"][1]],
                            [point["cost"][2] - point["cost"][0]]])
    return time_errors, cost_errors


if __name__ == '__main__':
    # store_data()
    plot()