

import sys
from pandas import read_csv
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# sample & path data indexes
x_index = 0
t_index = 1
in_edge_index = 2

def plot_motion_plan(filename):
    df_samples = read_csv('data/debug/samples' + filename + '.csv')

    # draw base graph
    for i in range(len(df_samples.index)):
        in_edges = [int(x) for x in df_samples.iloc[i, in_edge_index].split('#') if x]
        for e in in_edges:
            plt.plot([df_samples.iloc[i, x_index], df_samples.iloc[e, x_index]],
                     [df_samples.iloc[i, t_index], df_samples.iloc[e, t_index]], '.-k')


    # create handles for the legend
    sample_line = Line2D([], [], color='black', marker='.',
                              markersize=6, label='samples')

    plt.legend(handles=[sample_line])
    plt.xlabel("distance x")
    plt.ylabel("time t")
    plt.show()

if __name__ == '__main__':
    filename = sys.argv[1]

    plot_motion_plan(filename)