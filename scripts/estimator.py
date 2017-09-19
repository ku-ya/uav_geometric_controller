#!/usr/bin/env python
import numpy as np
from multiprocessing import Process
from threading import Thread
from numpy.linalg import norm
import time
import pprint
import sys

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseStamped
"""
For your work, the EKF or UKF framework for the position will be useful. Attitude estimation will be coded later on your program structure at a different work.

The control node will run as fast as possible, while getting state from the estimation node, which should run the propagation step at a predefined frequency. But, whenever a new measurement becomes available, the estimation node should run the corresponding measurement update scheme.

In short, the estimation node should run the propagation step regularly, and the measurement update occasionally.
"""

class Estimator(object):
    """
    Estimator node class for ROS
    """
    def __init__(self, uav_name = 'UAV', num_state = None, num_sens = None):
        self.nh = rospy.init_node('Estimator', anonymous=False)
        self.x = np.matrix(np.zeros(shape=(num_state, 1)))

        self.pose_pub = rospy.Publisher('pose_est', PoseStamped, queue_size=1)
        self.imu_pub = rospy.Publisher('imu_est', Imu, queue_size=1)
        self.imu_msg = Imu()
        self.pose_msg = PoseStamped()

        self.vicon_sub = rospy.Subscriber('/vicon/' + uav_name + "/pose", PoseStamped, self.vicon_callback)
        self.imu_sub = rospy.Subscriber(uav_name + "/imu", Imu, self.imu_callback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Exiting..."
        finally:
            print "\nKilling estimator node"
        pass

    def vicon_callback(self, data):
        # TODO update attitude
        self.pose_msg = data
        self.state_update()
        pass

    def imu_callback(self, data):
        self.imu_msg.angular_velocity = data.angular_velocity
        self.imu_msg.linear_acceleration = data.linear_acceleration
        self.state_update()
        self.state_predict()
        self.imu_pub.publish(self.imu_msg)
        self.pose_pub.publish(self.pose_msg)
        pass

    def state_predict(self):
        pass

    def state_update(self):
        pass


if __name__ == '__main__':
    estimator = Estimator(uav_name = 'Jetson',num_state = 12, num_sens=6)
