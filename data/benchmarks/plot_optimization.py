import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

resolution = 100
runtime = 10.0

def exp_func(x, k, y0, limit):
    return limit - ((limit - y0) * (np.e ** (-k * x)))

def get_plot_data(db):
    con = sqlite3.connect(db)
    cur = con.cursor()
    data = np.array(
        cur.execute("SELECT time, best_cost FROM {} WHERE best_cost <> 'inf'".format('progress')).fetchall())
    times, costs = np.split(data, 2, axis=1)
    times = times.flatten()
    costs = costs.flatten()
    max_cost = cur.execute("SELECT MAX(best_cost) FROM {} WHERE best_cost <> 'inf'".format('progress')).fetchall()[0][0]
    min_cost = cur.execute("SELECT MIN(best_cost) FROM {} WHERE best_cost <> 'inf'".format('progress')).fetchall()[0][0]
    min_time = cur.execute("SELECT MIN(time) FROM {} WHERE best_cost <> 'inf'".format('progress')).fetchall()[0][0]

    # create exponential fit
    popt, pcov = curve_fit(exp_func, times.tolist(), costs.tolist(), p0=(1.0, max_cost, min_cost))
    plot_times = np.linspace(min_time, runtime, resolution)
    yy = [exp_func(i, *popt) for i in plot_times]
    return plot_times, yy

def plot_optimization():

    star_xx, star_yy = get_plot_data('2/a1.db')
    st_xx, st_yy = get_plot_data('2/a1_spaceTime.db')

    plt.plot(star_xx, star_yy, color='blue', label='RRTStar')
    plt.plot(st_xx, st_yy, color='coral', label='SpaceTimeRRT')
    plt.legend()
    plt.xlabel("run time [s]")
    plt.ylabel("solution cost")

    plt.show()
    # plt.savefig('optimization.png')



if __name__ == '__main__':

    plot_optimization()
