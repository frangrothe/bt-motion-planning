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
col_obstacles = black

# sample & path data indexes
x_index = 0
t_index = 1
in_edge_index = 2
out_edge_index = 3
start_or_goal_index = 4

# x, t, width, height
goals = [[-1.1, 0, 0.1, 2.0], [1.0, 0, 0.1, 2.0]]
constraints = [[0.4, 0, 0.2, 0.6], [0.4, 0.9, 0.2, 1.1], [-0.8, 1.5, 0.2, 0.2], [-0.4, 0.4, 0.2, 0.4]]

filenames = [['11_samples.csv', '51_samples.csv'], ['82_samples.csv', 'final_samples.csv']]
solution_names = [['', ''], ['first_solution.csv', 'final_path.csv']]

def plot_all():
    plt.style.use('test.mplstyle')
    fig, axs = plt.subplots(2, 2, sharex='col', sharey='row')
    for i in range(2):
        for j in range(2):
            ax = axs[i][j]
            ax.set_xlim(-1.5, 1.5)
            ax.set_ylim(0.0, 2.0)
            if i == 1:
                ax.set_xlabel('position')
                ax.tick_params(labelbottom=False)
            if j == 0:
                ax.set_ylabel('time')
                ax.tick_params(labelleft=False)
            plot_statics(ax)
            samples = read_csv('data/cover_plot/' + filenames[i][j])
            plot_single(samples, ax)
            if solution_names[i][j] != '':
                path = read_csv('data/cover_plot/' + solution_names[i][j])
                plot_solution(path, ax)

    # plt.show()
    plt.savefig('cover_plot.pdf', format='pdf', dpi=300, bbox_inches='tight')


def plot_single(samples, ax):
    # draw start and goal tree
    for i in range(len(samples.index)):
        in_edges = [int(x) for x in samples.iloc[i, in_edge_index].split('#') if x]
        color = col_start_tree if samples.iloc[i, start_or_goal_index] == 1 else col_goal_tree
        for e in in_edges:
            ax.plot([samples.iloc[i, x_index], samples.iloc[e, x_index]],
                    [samples.iloc[i, t_index], samples.iloc[e, t_index]], '.-k', color=color)
        if len(in_edges) == 0:
            ax.scatter(samples.iloc[i, x_index], samples.iloc[i, t_index], marker='.', color=color, zorder=2)

def plot_solution(path, ax):
    ax.plot(path['x'].to_numpy(), path['time'].to_numpy(), 'D-', linewidth=2, color=col_solution, markersize=10)

def plot_statics(ax):
    for g in goals:
        ax.add_patch(Rectangle((g[0], g[1]), g[2], g[3], facecolor=col_goalregion, zorder=0))
    for c in constraints:
        ax.add_patch(Rectangle((c[0], c[1]), c[2], c[3], facecolor=col_obstacles, zorder=500))


if __name__ == '__main__':
    plot_all()
