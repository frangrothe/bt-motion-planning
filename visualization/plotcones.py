import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Patch
from matplotlib.lines import Line2D
from pandas import read_csv

# colorblind-friendly colors from the IBM Design Library
# https://davidmathlogic.com/colorblind/#%23648FFF-%23785EF0-%23DC267F-%23FE6100-%23FFB000
ibm_blue = '#648FFF'
ibm_violet = '#785EF0'
ibm_red = '#DC267F'
ibm_orange = '#FE6100'
ibm_yellow = '#FFB000'

col_start = ibm_blue
col_goal = ibm_red
col_intersection = ibm_violet

v = 1.5

def plot_cones():
    # draw start
    plt.style.use('test.mplstyle')
    plt.scatter(0, 0, marker='o', c=col_start, s=120, label='start', zorder=100)
    # draw goals
    plt.scatter([1, -1], [1, 0.85], marker='x', c=col_goal, s=120, label='goals', zorder=100)

    # draw start cone
    xs = np.linspace(-2, 2, 1000)
    ys = np.abs(xs) / v
    plt.fill_between(xs, ys, 1.1, color=col_start, alpha=0.5, label='start cone')

    # draw goal cones
    ysright = 1 - np.abs(xs - 1) / v
    # draw goal cone 2
    ysleft = 0.85 - np.abs(xs + 1) / v
    intercept = -0.1125
    for i in range(1000):
        if xs[i] <= intercept:
            ys[i] = ysleft[i]
        else:
            ys[i] = ysright[i]
    # ys[:50] = ysleft[:50]
    # ys[50:] = ysright[50:]
    plt.fill_between(xs, ys, -0.1, color=col_goal, alpha=0.5, label='goal cones')

    # draw intersection
    r = (1 + v) / 2
    l = -(0.85 * v + 1) / 2
    xs = np.linspace(l, r, 1000)
    y2 = np.abs(xs) / v
    ysright = 1 - np.abs(xs - 1) / v
    ysleft = 0.85 - np.abs(xs + 1) / v
    for i in range(1000):
        if xs[i] <= intercept:
            ys[i] = ysleft[i]
        else:
            ys[i] = ysright[i]
    # ys[:50] = ysleft[:50]
    # ys[50:] = ysright[50:]
    plt.fill_between(xs, ys, y2, color=col_intersection, alpha=0.9, label='intersection', zorder=50)

    plt.xlabel("position")
    plt.ylabel("time [s]")
    plt.xlim([-2, 2])
    plt.ylim([-0.1, 1.1])
    plt.subplots_adjust(bottom=0.65)
    plt.legend(bbox_to_anchor=(0.5, -0.3), loc="upper center",
           ncol=3)
    plt.show()
    # plt.savefig('cones.pdf', format='pdf', dpi=300, bbox_inches='tight')


if __name__ == '__main__':
    plot_cones()