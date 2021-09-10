import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import json

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


def get_from_progress(cur, times):
    count = cur.execute("SELECT COUNT(DISTINCT runid) FROM {}".format('progress')).fetchall()[0][0]
    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM (SELECT MIN(time) AS min_time "
            "FROM {0} WHERE best_cost<>'NONE' GROUP BY runid) min_time_table WHERE min_time < {1}".format(
                'progress', times[i])).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100
    return percentages


def get_percentages_first_solution(cur, times):
    count = cur.execute("SELECT COUNT(*) FROM {}".format('runs')).fetchall()[0][0]

    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM {0} WHERE time_first_solution < {1}".format('runs', times[i])).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100

    return percentages


def get_percentages_time(cur, times):
    id = cur.execute("SELECT id FROM {} WHERE name='geometric_RRTConnect'".format('plannerConfigs')).fetchall()[0][0]
    count = cur.execute("SELECT COUNT(*) FROM {} WHERE plannerid={}".format('runs', id)).fetchall()[0][0]

    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM {0} WHERE plannerid={2} AND time < {1}".format('runs', times[i], id)).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100

    return percentages


def store_data():
    with open(filepath + '.json', 'r') as jsonfile:
        data = json.load(jsonfile)

    # plot space time rrt
    times = np.logspace(np.log10(data["info"]["min_time"]["success"]), np.log10(data["info"]["max_time"]["success"]),
                        resolution)

    con = sqlite3.connect(filepath + '_spacetime.db')
    cur = con.cursor()
    space_time_rrt = get_percentages_first_solution(cur, times)
    # space_time_rrt = get_from_progress(cur)

    rrt_connect = []
    rrt_star = []

    rrtconnect_tb = data["info"]["rrtconnect_tb"]
    for i in range(len(rrtconnect_tb)):
        con = sqlite3.connect(filepath + '_' + str(rrtconnect_tb[i]) + '.db')
        cur = con.cursor()
        rrt_connect.append(get_percentages_time(cur, times))

    rrtstar_tb = data["info"]["rrtstar_tb"]
    for i in range(len(rrtstar_tb)):
        con = sqlite3.connect(filepath + '_' + str(rrtstar_tb[i]) + '.db')
        cur = con.cursor()
        rrt_star.append(get_from_progress(cur, times))

    data["spacetime"] = {
        "success": space_time_rrt.tolist()
    }
    for i in range(len(rrtconnect_tb)):
        data["rrtconnect" + str(rrtconnect_tb[i])] = {
            "success": rrt_connect[i].tolist()
        }

    for i in range(len(rrtstar_tb)):
        data["rrtstar" + str(rrtstar_tb[i])] = {
            "success": rrt_star[i].tolist()
        }

    with open(filepath + '.json', 'w') as jsonfile:
        json.dump(data, jsonfile, indent=4)


def plot():
    with open(filepath + '.json', 'r') as jsonfile:
        data = json.load(jsonfile)

    min_time = data["info"]["min_time"]["success"]
    max_time = data["info"]["max_time"]["success"]
    times = np.logspace(np.log10(min_time), np.log10(max_time), resolution)

    plt.style.use('../../test.mplstyle')
    plt.xscale('log')
    plt.xlim(min_time, max_time)
    plt.yscale('linear')
    plt.ylim(0.0, 100.0)

    space_time_rrt = data["spacetime"]["success"]
    plt.plot(times, space_time_rrt, color=col_space_time_rrt, linestyle='-', label='ST-RRT*')

    linestyles = ['-', '--', 'dotted']
    rrtconnect_tb = data["info"]["rrtconnect_tb"]
    for i in range(len(rrtconnect_tb)):
        rrt_connect = data["rrtconnect" + str(rrtconnect_tb[i])]["success"]
        plt.plot(times, rrt_connect, color=col_rrt_connect, linestyle=linestyles[i],
                 label='RRTConnect ' + str(rrtconnect_tb[i]))
    rrtstar_tb = data["info"]["rrtstar_tb"]
    for i in range(len(rrtstar_tb)):
        rrt_star = data["rrtstar" + str(rrtstar_tb[i])]["success"]
        plt.plot(times, rrt_star, color=col_rrt_star, linestyle=linestyles[i], label='RRT* ' + str(rrtstar_tb[i]))
    plt.legend()
    plt.grid(True, which="both", ls='--')
    plt.xlabel("run time [s]")
    plt.ylabel("success [\%]")

    # plt.show()
    plt.savefig('narrow_success.pdf', format='pdf', dpi=300, bbox_inches='tight')


def store_info():
    dict = {
        "min_time": {
            "success": 0.0001,
            "optimization": 0.001
        },
        "max_time": {
            "success": 2.0,
            "optimization": 2.0
        },
        "max_cost": 4,
        "ci_left": 39,
        "ci_right": 59,
        "rrtconnect_tb": [1, 2, 4],
        "rrtstar_tb": [1, 2, 4]
    }
    data = {}
    if os.path.isfile(filepath + '.json'):
        with open(filepath + '.json', 'r') as jsonfile:
            data = json.load(jsonfile)

    data["info"] = dict

    with open(filepath + '.json', 'w') as jsonfile:
        json.dump(data, jsonfile, indent=4)


if __name__ == '__main__':
    # store_info()
    # store_data()
    plot()

    # tables = cur.execute("SELECT name FROM sqlite_master WHERE type='table' ORDER BY name").fetchall()
    # for table in tables:
    #     print("\nNAME: " + table[0] + "\n")
    #     cur.execute("SELECT * FROM {}".format(table[0])).fetchall()
    #     names = list(map(lambda x: x[0], cur.description))
    #     print(names)
    #
    # print(4 * '\n')
    # planners = cur.execute("SELECT id, name FROM {}".format('plannerConfigs')).fetchall()
    # for planner in planners:
    #     print("Planner {} has ID {}".format(planner[1], planner[0]))
    #
    # print(4 * '\n')
    # experiments = cur.execute("SELECT id, name, timelimit, runcount FROM {}".format('experiments')).fetchall()
    # for experiment in experiments:
    #     print(experiment)
    #
    # print(4 * '\n')
    # runs = cur.execute("SELECT id, experimentid, plannerid, time, simplified_correct_solution FROM {}".format('runs')).fetchall()
    # for run in runs:
    #     print(run)
    #
    # print(4 * '\n')
    # count = cur.execute("SELECT COUNT(DISTINCT runid) FROM {}".format('progress')).fetchall()
    # print(count)
    # best_times = cur.execute(
    #     "SELECT runid, MIN(time) FROM {} WHERE best_cost<>'NONE' GROUP BY runid".format('progress')).fetchall()
    # print(best_times)
    #
    # percentage = cur.execute(
    #     "SELECT COUNT(*) FROM (SELECT MIN(time) AS min_time FROM {} WHERE best_cost<>'NONE' GROUP BY runid) min_time_table WHERE min_time < 2.0".format(
    #         'progress')).fetchall()
    # print(percentage)
