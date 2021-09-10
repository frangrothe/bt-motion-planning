
import matplotlib as mpl
# Use the pgf backend (must be set before pyplot imported)
# mpl.use('pgf')
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

# sample & path data indexes
x_index = 0
t_index = 1
in_edge_index = 2

# constraint & goal region indexes
x_lb_index = 0
x_ub_index = 1
t_lb_index = 2
t_ub_index = 3

time_ub = 1024

def plot_samples(filename, ax):
    df_samples = read_csv('data/' + filename + '/samples.csv')
    for i in range(len(df_samples.index)):
        in_edges = [int(x) for x in df_samples.iloc[i, in_edge_index].split('#') if x]
        for e in in_edges:
            ax.plot([df_samples.iloc[i, x_index], df_samples.iloc[e, x_index]],
                     [df_samples.iloc[i, t_index], df_samples.iloc[e, t_index]], '.-k', markersize=1, lw=0.5)



def plot_narrow():
    plt.style.use('test.mplstyle')
    # plt.figure(figsize=(8, 8))
    f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)

    plot_samples('narrow_nonuniform', ax1)
    plot_samples('narrow_uniform', ax2)

    # draw solution
    df_path = read_csv('data/' + 'narrow_uniform' + '/path.csv')
    ax2.plot(df_path['x'].to_numpy(), df_path['time'].to_numpy(), 'D-', linewidth=2, color=ibm_red, markersize=1)

    # draw constraints
    ax1.add_patch(Rectangle((0.4, 0), 0.2, 300, facecolor=ibm_blue))
    ax1.add_patch(Rectangle((0.4, 305), 0.2, time_ub - 305, facecolor=ibm_blue))
    ax2.add_patch(Rectangle((0.4, 0), 0.2, 300, facecolor=ibm_blue))
    ax2.add_patch(Rectangle((0.4, 305), 0.2, time_ub - 305, facecolor=ibm_blue))

    # draw goal
    ax1.add_patch(Rectangle((1, 0), 0.05, time_ub, facecolor=ibm_yellow, zorder=500))
    ax2.add_patch(Rectangle((1, 0), 0.05, time_ub, facecolor=ibm_yellow, zorder=500))

    # draw start
    ax1.scatter(0, 0, c=ibm_orange, marker='s', s=50, zorder=500)
    ax2.scatter(0, 0, c=ibm_orange, marker='s', s=50, zorder=500)

    ax1.set_title('Nonuniform')
    ax2.set_title('Uniform')

    ax1.set_xlim(-2, 2)
    ax1.set_ylim(0, time_ub)
    ax1.set_xlabel('position')
    ax1.set_ylabel('time [s]')
    ax2.set_xlim(-2, 2)
    ax2.set_xlabel('position')

    # create handles for the legend
    sample_line = Line2D([], [], color='black', marker='.',
                         markersize=6, label='samples')
    path_line = Line2D([], [], color=ibm_red, marker='D',
                       markersize=6, label='solution path')
    start_line = Line2D([], [], color=ibm_orange, marker='s',
                         markersize=6, label='start', ls='')
    constraint_patch = Patch(color=ibm_blue, label='constraints')
    goal_patch = Patch(color=ibm_yellow, label='goal region')
    plt.legend(handles=[sample_line, path_line, start_line, constraint_patch, goal_patch])
    ax1.grid(True, which="both", ls='--')
    ax2.grid(True, which="both", ls='--')


    # plt.show()
    plt.savefig('narrow_large.pdf', format='pdf', dpi=300, bbox_inches='tight')


if __name__ == '__main__':
    plot_narrow()