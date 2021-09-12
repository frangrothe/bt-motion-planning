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
black = '#000000'

col_goal_tree = ibm_red
col_start_tree = ibm_blue
col_solution = ibm_orange
col_goalregion = ibm_yellow
col_timebound = ibm_violet
col_obstacles = black

# sample & path data indexes
x_index = 0
t_index = 1
in_edge_index = 2
out_edge_index = 3
start_or_goal_index = 4

# x, t, width, height
goals = [[-1.1, 0, 0.1, 4.5], [1.0, 0, 0.1, 4.5]]
constraints = [[0.4, 0, 0.2, 2.4], [0.4, 3.0, 0.2, 1.0], [-0.8, 2.6, 0.2, 0.4], [-0.4, 0.4, 0.2, 1.6]]

filenames = [['100_samples.csv', '251_samples.csv'], ['300_samples.csv', 'final_samples.csv']]
solution_names = [['', ''], ['first_solution.csv', 'final_path.csv']]
labels = [['a', 'b'], ['c', 'd']]
timebounds = [[2, 4], [3.71, 1]]

def plot_all():
    plt.style.use('test.mplstyle')
    fig, axs = plt.subplots(2, 2, sharex='col', sharey='row')
    for i in range(2):
        for j in range(2):
            ax = axs[i][j]
            ax.set_xlim(-1.5, 1.5)
            ax.set_ylim(0.0, 4.5)
            ax.set_title(labels[i][j])
            if i == 1:
                ax.set_xlabel('position')
                ax.tick_params(labelbottom=False)
            if j == 0:
                ax.set_ylabel('time')
                ax.tick_params(labelleft=False)

            # draw time bounds
            t = timebounds[i][j]
            ax.plot([-1.5, 1.5], [t, t], c=col_timebound, zorder=1000, ls='--')
            plot_statics(ax)
            samples = read_csv('data/cover_plot/' + filenames[i][j])
            plot_single(samples, ax, 1.0, 50)
            if solution_names[i][j] != '':
                path = read_csv('data/cover_plot/' + solution_names[i][j])
                plot_solution(path, ax)
            if i == 1 and j == 1:
                plot_pruned_nodes(ax)
            if i == 1 and j == 0:
                samples = read_csv('data/cover_plot/' + str(1) + '_prePruning.csv')
                plot_single(samples, ax, 0.1, 1)

    # plt.show()
    plt.savefig('cover_plot.pdf', format='pdf', dpi=300, bbox_inches='tight')

def plot_pruned_nodes(ax):
    to_draw = [1, 3, 6]
    for n in to_draw:
        samples = read_csv('data/cover_plot/' + str(n) + '_prePruning.csv')
        plot_single(samples, ax, 0.1, 1)

def plot_single(samples, ax, alpha, zorder):
    # draw start and goal tree
    for i in range(len(samples.index)):
        in_edges = [int(x) for x in samples.iloc[i, in_edge_index].split('#') if x]
        color = col_start_tree if samples.iloc[i, start_or_goal_index] == 1 else col_goal_tree
        for e in in_edges:
            ax.plot([samples.iloc[i, x_index], samples.iloc[e, x_index]],
                    [samples.iloc[i, t_index], samples.iloc[e, t_index]], '.-', color=color, alpha=alpha, zorder=zorder, lw=0.5, ms=0.5)
        # if len(in_edges) == 0:
        #     ax.scatter(samples.iloc[i, x_index], samples.iloc[i, t_index], marker='.', color=color, zorder=zorder, alpha=alpha)

def plot_solution(path, ax):
    ax.plot(path['x'].to_numpy(), path['time'].to_numpy(), linewidth=4, color=col_solution, markersize=0.5, zorder=500)

def plot_statics(ax):
    for g in goals:
        ax.add_patch(Rectangle((g[0], g[1]), g[2], g[3], facecolor=col_goalregion, zorder=0))
    for c in constraints:
        ax.add_patch(Rectangle((c[0], c[1]), c[2], c[3], facecolor=col_obstacles, zorder=500))




if __name__ == '__main__':
    plot_all()
