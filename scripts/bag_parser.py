#!/usr/bin/env python

import rosbag
from std_msgs.msg import Int32, String
import numpy as np
import pdb
import yaml
import argparse

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def load_bag_file(filename):

    bag = rosbag.Bag(filename, 'r')

    info_dict = yaml.load(bag._get_yaml_info())

    topics = bag.get_type_and_topic_info()[1].keys()
    types = []

    for ii in range(0, len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[ii][0])

    return bag, info_dict, topics, types


def read_bag_file(filename):
    # pdb.set_trace()
    bag, info_dict, all_topics, all_types = load_bag_file(filename)

    drone_var_topic = '/drone_variable'

    drone_var_msg_type = bag.get_type_and_topic_info()[1].get(drone_var_topic)[0]
    num_drone_var_msg = bag.get_type_and_topic_info()[1].get(drone_var_topic)[1]
    drone_var_msg_freq = bag.get_type_and_topic_info()[1].get(drone_var_topic)[2]

    print("Printing topic - %s " % drone_var_topic)
    print("Message type - %s" % drone_var_msg_type)
    print("Message frequency - %s" % drone_var_msg_freq)
    print("Number of messages - %g" % num_drone_var_msg)

    print("")
    print("Now reading the data")

    # setup arrays for data
    time_array = np.zeros((num_drone_var_msg,1))
    xd_array = np.zeros((num_drone_var_msg,3))
    xd_dot_array = np.zeros((num_drone_var_msg,3))
    xd_ddot_array = np.zeros((num_drone_var_msg,3))
    xd_array = np.zeros((num_drone_var_msg,3))
    x_v_array = np.zeros((num_drone_var_msg,3))
    v_v_array = np.zeros((num_drone_var_msg,3))
    IMU_array = np.zeros((num_drone_var_msg,3))
    rpy_array = np.zeros((num_drone_var_msg,3))
    xd_array = np.zeros((num_drone_var_msg,3))
    ex_array = np.zeros((num_drone_var_msg,3))
    ev_array = np.zeros((num_drone_var_msg,3))
    eR_array = np.zeros((num_drone_var_msg,3))
    eW_array = np.zeros((num_drone_var_msg,3))
    f_array = np.zeros((num_drone_var_msg,1))
    f_motor_array = np.zeros((num_drone_var_msg,4))
    thr_array = np.zeros((num_drone_var_msg,4))
    M_array = np.zeros((num_drone_var_msg,3))
    gainX_array = np.zeros((num_drone_var_msg,4))
    gainR_array = np.zeros((num_drone_var_msg,4))
    dt_vicon_imu_array = np.zeros((num_drone_var_msg,1))

    drone_var_index = 0

    for topic, msg, t in bag.read_messages(topics=[drone_var_topic]):
        # pdb.set_trace()
        time_array[drone_var_index,0] = t.to_sec()
        xd_array[drone_var_index,:] = np.array([msg.xd.x, msg.xd.y, msg.xd.z])
        x_v_array[drone_var_index,:] = np.array([msg.x_v.x, msg.x_v.y, msg.x_v.z])
        IMU_array[drone_var_index,:] = np.array([msg.IMU.x, msg.IMU.y, msg.IMU.z])
        rpy_array[drone_var_index,:] = np.array([msg.rpy.x, msg.rpy.y, msg.rpy.z])
        v_v_array[drone_var_index,:] = np.array([msg.v_v.x, msg.v_v.y, msg.v_v.z])
        ex_array[drone_var_index,:] = np.array([msg.ex.x, msg.ex.y, msg.ex.z])
        ev_array[drone_var_index,:] = np.array([msg.ev.x, msg.ev.y, msg.ev.z])
        eW_array[drone_var_index,:] = np.array([msg.eW.x, msg.eW.y, msg.eW.z])
        f_array[drone_var_index,:] = np.array([msg.force])
        dt_vicon_imu_array[drone_var_index,:] = np.array([msg.dt_vicon_imu])

        drone_var_index += 1


    bag.close()

    # Time shift the time_array by the starting epoch
    # vicon_time_array = (vicon_time_array - vicon_time_array[0])
    # xd_time_array = (xd_time_array - xd_time_array[0])

    # create some figures
    mpl.rcParams['legend.fontsize'] = 10

    fig_traj = plt.figure()
    ax = fig_traj.gca(projection='3d')

    ax.plot(x_v_array[:,0],x_v_array[:,1],x_v_array[:,2])
    ax.set_zlim(0, 3)

    fig_comp = plt.figure()
    plt.subplot(311)
    plt.plot(time_array[:],x_v_array[:,0],'b', label='Vicon')
    plt.plot(time_array[:], xd_array[:,0],'r', label='Desired')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel('X Position (m)')
    plt.legend()

    plt.subplot(312)
    plt.plot(time_array[:],x_v_array[:,1],'b', label='Vicon')
    plt.plot(time_array[:], xd_array[:,1],'r', label='Desired')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel('Y Position (m)')
    plt.legend()

    plt.subplot(313)
    plt.plot(time_array[:],x_v_array[:,2],'b', label='Vicon')
    plt.plot(time_array[:], xd_array[:,2],'r', label='Desired')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel('Z Position (m)')
    plt.legend()

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Parse a ROS Bag file and plot.')
    parser.add_argument('input_file', help='Input ROS Bag file')
    args = parser.parse_args()

    read_bag_file(args.input_file)
