#!/usr/bin/env python
import numpy as np
import rospy
from uav_geometric_controller.msg import trajectory

import matplotlib.pyplot as plt
import time
import threading
import random

data = []

# This just simulates reading from a socket.
def data_listener():
    while True:
        time.sleep(1)
        data.append(random.random())

if __name__ == '__main__':
    thread = threading.Thread(target=data_listener)
    thread.daemon = True
    thread.start()
    #
    # initialize figure
    plt.figure()
    ln, = plt.plot([])
    plt.ion()
    plt.show()
    while True:
        plt.pause(1)
        ln.set_xdata(range(len(data)))
        ln.set_ydata(data)
        plt.draw()
