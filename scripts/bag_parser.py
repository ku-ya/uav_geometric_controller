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
    
    bag, info_dict, topics, types = load_bag_file(filename)

    num_vicon_msg = info_dict['topics'][-3]['messages']
    vicon_topic = info_dict['topics'][-3]['topic']
    vicon_msg_type = info_dict['topics'][-3]['type']
    vicon_msg_freq = info_dict['topics'][-3]['frequency']

    num_xd_msg = info_dict['topics'][-1]['messages']
    xd_topic = info_dict['topics'][-1]['topic']
    xd_msg_type = info_dict['topics'][-1]['type']
    xd_msg_freq = info_dict['topics'][-1]['frequency']

    print("Printing topic - %s " % vicon_topic)
    print("Message type - %s" % vicon_msg_type)
    print("Message frequency - %s" % vicon_msg_freq)
    print("Number of messages - %g" % num_vicon_msg)

    print("")

    print("Printing topic - %s " % xd_topic)
    print("Message type - %s" % xd_msg_type)
    print("Message frequency - %s" % xd_msg_freq)
    print("Number of messages - %g" % num_xd_msg)

    print("")

    print("Now reading the data")

    # setup arrays for vicon data
    vicon_time_array = np.zeros((num_vicon_msg,1))
    vicon_pos_array = np.zeros((num_vicon_msg,3))
    # 4 element quaterion with scalar element in the last column
    vicon_quat_array = np.zeros((num_vicon_msg,4))

    # setup array for xd
    xd_time_array = np.zeros((num_xd_msg,1))
    xd_pos_array = np.zeros((num_xd_msg,3))

    vicon_index = 0
    xd_index = 0
    for topic, msg, t in bag.read_messages(topics=[vicon_topic,xd_topic]):
        
        if topic == vicon_topic:
            vicon_time_array[vicon_index,0] = t.to_sec()
            vicon_pos_array[vicon_index,:] = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
            vicon_quat_array[vicon_index,:] = np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])

            vicon_index += 1
        elif topic == xd_topic:

            xd_time_array[xd_index] = t.to_sec()
            xd_pos_array[xd_index,:] = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            xd_index += 1


    # Time shift the time_array by the starting epoch
    vicon_time_array = (vicon_time_array - vicon_time_array[0])
    xd_time_array = (xd_time_array - xd_time_array[0])

    # create some figures
    mpl.rcParams['legend.fontsize'] = 10

    fig_traj = plt.figure()
    ax = fig_traj.gca(projection='3d')

    ax.plot(vicon_pos_array[:,0],vicon_pos_array[:,1],vicon_pos_array[:,2])

    fig_comp = plt.figure()
    plt.subplot(311)
    plt.plot(vicon_time_array[:],vicon_pos_array[:,0],'b', label='Vicon')
    plt.plot(xd_time_array[:], xd_pos_array[:,0],'r', label='Desired')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel('X Position (m)')
    plt.legend()

    plt.subplot(312)
    plt.plot(vicon_time_array[:],vicon_pos_array[:,1],'b', label='Vicon')
    plt.plot(xd_time_array[:], xd_pos_array[:,1],'r', label='Desired')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel('Y Position (m)')
    plt.legend()

    plt.subplot(313)
    plt.plot(vicon_time_array[:],vicon_pos_array[:,2],'b', label='Vicon')
    plt.plot(xd_time_array[:], xd_pos_array[:,2],'r', label='Desired')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel('Z Position (m)')
    plt.legend()

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Parse a ROS Bag file and plot.')
    parser.add_argument('input_file', help='Input ROS Bag file')
    args = parser.parse_args()

    read_bag_file(args.input_file)