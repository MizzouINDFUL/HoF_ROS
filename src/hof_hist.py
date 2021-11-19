import rospy

from std_msgs.msg import Float64MultiArray

import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

global hist0, hist2

def update0(msg):
    global hist0
    data = np.asarray(msg.data)
    # print(data)
    hist0 = data

def update2(msg):
    global hist2
    data = np.asarray(msg.data)
    # print(data)
    data[np.isinf(data)] = np.nan
    hist2 = data

def on_close(event):
    rospy.signal_shutdown("Figure closed")

if __name__ == "__main__":

    rospy.init_node('hof_hist')
    rate = rospy.Rate(10)

    N = 181

    x = np.linspace(0, 360, N)

    hist0 = np.zeros(181)
    hist2 = np.zeros(181)

    plt.ion()

    fig, ax = plt.subplots(2, 1)
    fig.canvas.mpl_connect('close_event', on_close)
    plt.subplots_adjust(hspace=0.5)

    ax[0].set_title('F0 Histogram')
    ax[1].set_title('F2 Histogram')

    line0, = ax[0].plot(x, hist0)
    line2, = ax[1].plot(x, hist2)

    rospy.Subscriber("/f0_histogram", Float64MultiArray, update0)
    rospy.Subscriber("/f2_histogram", Float64MultiArray, update2)

    while not rospy.is_shutdown():

        line0.set_ydata(hist0)
        ax[0].set_ylim([0, hist0.max()])

        line2.set_ydata(hist2)
        ax[1].set_ylim([0, np.nanmax(hist2)])

        fig.canvas.draw()
        fig.canvas.flush_events()

        rate.sleep()