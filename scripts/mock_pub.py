#!/usr/bin/env python

import rospy
from uav_geometric_controller.msg import trajectory, states
import numpy as np

def talker():
    pub = rospy.Publisher('Jetson/uav_states', states, queue_size=10)
    rospy.init_node('talker', anonymous=False)
    rate = rospy.Rate(100) # 10hz
    data = states()
    time = 0.01
    while not rospy.is_shutdown():
        data.x_v.x = np.sin(10*time)
        data.x_v.y = np.sin(15*time)
        data.x_v.z = np.sin(20*time)
        data.eW.x = np.sin(10*time)
        data.eW.y = np.sin(15*time)
        data.eW.z = np.sin(20*time)
        data.eR.x = np.sin(10*time) + np.random.rand() - 0.5
        data.eR.y = np.sin(15*time)
        data.eR.z = np.sin(20*time)
        data.moment.x = np.sin(10*time)
        data.moment.y = np.sin(15*time) + np.random.rand() - 0.5
        data.moment.z = np.sin(20*time)
        pub.publish(data)
        time += 0.01
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
