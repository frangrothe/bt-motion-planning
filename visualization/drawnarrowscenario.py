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

start_color = ibm_orange
goal_color = ibm_yellow
obstacle_color = ibm_blue

def draw():
    plt.style.use('test.mplstyle')
    plt.figure(figsize=(8,8))
    plt.xlim([-0.5, 1.5])
    plt.ylim([0, 4.0])
    plt.tick_params(
        axis='x',  # changes apply to the x-axis
        which='both',  # both major and minor ticks are affected
        bottom=False,  # ticks along the bottom edge are off
        top=False,  # ticks along the top edge are off
        labelbottom=False)  # labels along the bottom edge are off
    plt.tick_params(
        axis='y',  # changes apply to the x-axis
        which='both',  # both major and minor ticks are affected
        bottom=False,  # ticks along the bottom edge are off
        top=False,  # ticks along the top edge are off
        left=False,
        labelleft=False)  # labels along the bottom edge are off
    current_axis = plt.gca()

    # x, y, width, height
    obs = [[0.4, 0.0, 0.2, 0.4], [0.4, 0.6, 0.2, 0.4], [0.4, 1.4, 0.2, 0.4], [0.4, 2.6, 0.2, 1.4]]
    for o in obs:
        current_axis.add_patch(Rectangle((o[0], o[1]), o[2], o[3], facecolor=obstacle_color))

    plt.scatter(0, 0, marker='s', c=start_color, s=120, zorder=100)
    current_axis.add_patch(Rectangle((1, 0), 0.05, 4, facecolor=goal_color))

    # create handles for the legend
    constraint_patch = Patch(color=obstacle_color, label='obstacles')
    goal_patch = Patch(color=ibm_yellow, label='goal region')
    start_line = Line2D([], [], color=ibm_orange, marker='s',
                        markersize=6, label='start', ls='')

    # plt.legend(handles=[start_line, constraint_patch, goal_patch], loc='upper left')
    # plt.show()
    plt.savefig('narrow_scenario.pdf', format='pdf', dpi=300, bbox_inches='tight')


if __name__ == '__main__':
    draw()