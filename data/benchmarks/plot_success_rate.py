import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# general parameters
resolution = 100
times = np.linspace(0.0, 10.0, resolution)


def exp_func(x, k):
    return 100 - (100 * (np.e ** (-k * x)))

def get_from_progress(cur):
    count = cur.execute("SELECT COUNT(DISTINCT runid) FROM {}".format('progress')).fetchall()[0][0]

    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM (SELECT MIN(time) AS min_time "
            "FROM {0} WHERE best_cost<>'NONE' GROUP BY runid) min_time_table WHERE min_time < {1}".format(
                'progress', times[i])).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100

    # create exponential fit
    popt, pcov = curve_fit(exp_func, times, percentages)
    yy = [exp_func(i, *popt) for i in times]

    return yy

def get_from_runs(cur):
    count = cur.execute("SELECT COUNT(*) FROM {} WHERE plannerid=1".format('runs')).fetchall()[0][0]

    percentages = np.empty(len(times))
    for i in range(len(times)):
        percentage = cur.execute(
            "SELECT COUNT(*) FROM {0} WHERE plannerid=1 AND time < {1}".format('runs', times[i])).fetchall()
        percentages[i] = (percentage[0][0] / count) * 100
    popt, pcov = curve_fit(exp_func, times, percentages)
    yy = [exp_func(i, *popt) for i in times]

    return yy

def plot_succcess():

    con = sqlite3.connect('2/a1.db')
    cur = con.cursor()
    connect_yy = get_from_runs(cur)
    star_yy = get_from_progress(cur)

    con = sqlite3.connect('2/a1_spaceTime.db')
    cur = con.cursor()
    st_yy = get_from_progress(cur)

    plt.plot(times, star_yy, color='blue', label='RRTStar')
    plt.plot(times, connect_yy, color='green', label='RRTConnect')
    plt.plot(times, st_yy, color='coral', label='SpaceTimeRRT')
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