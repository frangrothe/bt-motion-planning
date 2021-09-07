import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import json

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
min_time = 0.01
max_time = 30

# general parameters
resolution = 200
times = np.logspace(np.log10(min_time), np.log10(max_time), resolution)

def get_from_progress(cur):
    count = cur.execute("SELECT COUNT(DISTINCT runid) FROM {}".format('progress')).fetchall()[0][0]
    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM (SELECT MIN(time) AS min_time "
            "FROM {0} WHERE best_cost<>'NONE' GROUP BY runid) min_time_table WHERE min_time < {1}".format(
                'progress', times[i])).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100
    return percentages

def get_percentages_first_solution(cur):
    count = cur.execute("SELECT COUNT(*) FROM {}".format('runs')).fetchall()[0][0]

    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM {0} WHERE time_first_solution < {1}".format('runs', times[i])).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100

    return percentages

def get_percentages_time(cur):

    id = cur.execute("SELECT id FROM {} WHERE name='geometric_RRTConnect'".format('plannerConfigs')).fetchall()[0][0]
    count = cur.execute("SELECT COUNT(*) FROM {} WHERE plannerid={}".format('runs', id)).fetchall()[0][0]

    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM {0} WHERE plannerid={2} AND time < {1}".format('runs', times[i], id)).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100

    return percentages

def store_data():
    # plot space time rrt
    con = sqlite3.connect(filepath + '_spacetime.db')
    cur = con.cursor()
    space_time_rrt = get_percentages_first_solution(cur)

    # plot rrt connect and rrt star
    con = sqlite3.connect(filepath + '_8.db')
    cur = con.cursor()
    rrt_connect8 = get_percentages_time(cur)
    rrt_star8 = get_from_progress(cur)

    # plot rrt connect and rrt star
    con = sqlite3.connect(filepath + '_32.db')
    cur = con.cursor()
    rrt_connect32 = get_percentages_time(cur)
    rrt_star32 = get_from_progress(cur)

    con = sqlite3.connect(filepath + '_128.db')
    cur = con.cursor()
    rrt_connect128 = get_percentages_time(cur)
    rrt_star128 = get_from_progress(cur)

    data = {
        "spacetime" : {
            "success" : space_time_rrt.tolist()
        },
        "rrtconnect8" : {
            "success" : rrt_connect8.tolist()
        },
        "rrtconnect32": {
            "success": rrt_connect32.tolist()
        },
        "rrtconnect128": {
            "success": rrt_connect128.tolist()
        },
        "rrtstar8": {
            "success": rrt_star8.tolist()
        },
        "rrtstar32": {
            "success": rrt_star32.tolist()
        },
        "rrtstar128": {
            "success": rrt_star128.tolist()
        }
    }

    with open(filepath + '.json','w') as jsonfile:
        json.dump(data, jsonfile, indent=4)


def plot():
    with open(filepath + '.json','r') as jsonfile:
        data = json.load(jsonfile)

    space_time_rrt = data["spacetime"]["success"]
    rrt_connect8 = data["rrtconnect8"]["success"]
    rrt_star8 = data["rrtstar8"]["success"]
    rrt_connect32 = data["rrtconnect32"]["success"]
    rrt_star32 = data["rrtstar32"]["success"]
    rrt_connect128 = data["rrtconnect128"]["success"]
    rrt_star128 = data["rrtstar128"]["success"]

    plt.xscale('log')
    plt.xlim(min_time, max_time)

    plt.yscale('linear')
    plt.ylim(0.0, 100.0)

    plt.plot(times, space_time_rrt, color=col_space_time_rrt, linestyle='-', label='SpaceTimeRRT')
    plt.plot(times, rrt_connect8, color=col_rrt_connect, linestyle='-', label='RRTConnect 8')
    plt.plot(times, rrt_connect32, color=col_rrt_connect, linestyle='--', label='RRTConnect 32')
    plt.plot(times, rrt_connect128, color=col_rrt_connect, linestyle='dotted', label='RRTConnect 128')
    plt.plot(times, rrt_star8, color=col_rrt_star, linestyle='-', label='RRTStar 8')
    plt.plot(times, rrt_star32, color=col_rrt_star, linestyle='--', label='RRTStar 32')
    plt.plot(times, rrt_star128, color=col_rrt_star, linestyle='dotted', label='RRTStar 128')

    plt.legend()
    plt.xlabel("run time [s]")
    plt.ylabel("success [%]")

    # plt.show()
    plt.savefig('dim8_success.png')


if __name__ == '__main__':

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