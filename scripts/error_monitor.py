#!/usr/bin/env python
import numpy as np
import rospy
from uav_geometric_controller.msg import trajectory

import matplotlib.pyplot as plt
import time
import threading
import random
from collections import deque

eW = deque([0]*10, 100)

# This just simulates reading from a socket.
def callback(data):

    pass

if __name__ == '__main__':
    rospy.init_node('error_plot', anonymous=True)
    rospy.Subscriber("err", trajectory, callback)

    # thread = threading.Thread(target=data_listener)
    # thread.daemon = True
    # thread.start()
    # time.sleep(1)
    #
    # initialize figure
    fig = plt.figure()
    ax_eW = plt.subplot(311)
    ax_eR = plt.subplot(312)
    ax_M = plt.subplot(313)
    ln, = ax_eR.plot([],'rs')
    plt.ion()
    plt.show()
    while True:
        plt.pause(0.1)
        # ln.set_xdata(range(len(data)))
        # ln.set_ydata(data)
        ax_eW.cla()
        ax_eW.set_ylabel('eW')
        ax_eW.plot(eW)
        ax_eR.cla()
        ax_eR.set_ylabel('eR')
        ax_eR.plot(eR)
        ax_M.cla()
        ax_M.set_ylabel('M')
        ax_M.plot(M)
        plt.draw()
