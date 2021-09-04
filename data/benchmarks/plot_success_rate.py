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

def plot_succcess():
    # plot space time rrt
    con = sqlite3.connect('2/a1_spacetime.db')
    cur = con.cursor()
    space_time_rrt = get_percentages_first_solution(cur)

    plt.xscale('log')
    plt.xlim(0.01, 5.0)

    plt.yscale('linear')
    plt.ylim(0.0, 100.0)

    plt.scatter(times, space_time_rrt, color=col_space_time_rrt, label='SpaceTimeRRT')

    plt.legend()
    plt.xlabel("run time [s]")
    plt.ylabel("chance to find solution [%]")

    plt.show()
    # plt.savefig('successes.png')



if __name__ == '__main__':

    plot_succcess()

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