import math
import sys
from pandas import read_csv
import matplotlib.pyplot as plt


# colorblind-friendly colors from the IBM Design Library
# https://davidmathlogic.com/colorblind/#%23648FFF-%23785EF0-%23DC267F-%23FE6100-%23FFB000
ibm_blue = '#648FFFaa'
ibm_violet = '#785EF0'
ibm_red = '#DC267F'
ibm_orange = '#FE6100'
ibm_yellow = '#FFB000'

def plot_motion_plan(filename):
    df_path = read_csv('data/' + filename + '/path.csv')

    # draw base graph
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # draw solution path
    ax.plot(df_path['x'].to_numpy(), df_path['y'].to_numpy(), df_path['time'].to_numpy(),
            'D-', linewidth=2, color=ibm_red, zorder=8000, label='solution')

    # draw yaw lines
    # for i in range(len(df_path.index)):
    #     x = df_path.iloc[i, 0]
    #     y = df_path.iloc[i, 1]
    #     yaw = df_path.iloc[i, 2]
    #     t = df_path.iloc[i, 3]
    #     newx = x + math.cos()

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')

    plt.show()


# Call with csv file to plot as command line argument
if __name__ == '__main__':
    filename = sys.argv[1]
    plot_motion_plan(filename)