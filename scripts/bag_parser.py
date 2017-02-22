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
    print('')
    print('Loading bag file ...')

    bag = rosbag.Bag(filename, 'r')

    info_dict = yaml.load(bag._get_yaml_info())

    topics = bag.get_type_and_topic_info()[1].keys()
    types = []

    for ii in range(0, len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[ii][0])

    print('Bag file loaded...!')
    return bag, info_dict, topics, types



def read_bag_file(filename):
    # pdb.set_trace()
    bag, info_dict, all_topics, all_types = load_bag_file(filename)

    drone_var_topic = '/drone_variable'

    drone_var_msg_type = bag.get_type_and_topic_info()[1].get(drone_var_topic)[0]
    num_drone_var_msg = bag.get_type_and_topic_info()[1].get(drone_var_topic)[1]
    drone_var_msg_freq = bag.get_type_and_topic_info()[1].get(drone_var_topic)[2]

    print('Printing topic - %s ' % drone_var_topic)
    print('Message type - %s' % drone_var_msg_type)
    print('Message frequency - %s' % drone_var_msg_freq)
    print('Number of messages - %g' % num_drone_var_msg)

    print('')
    print('Reading the data ...')

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
    dt_vicon_array = np.zeros((num_drone_var_msg,1))

    drone_var_index = 0

    for topic, msg, t in bag.read_messages(topics=[drone_var_topic]):
        # pdb.set_trace()
        time_array[drone_var_index,0] = t.to_sec()
        xd_array[drone_var_index,:] = np.array([msg.xd.x, msg.xd.y, msg.xd.z])
        xd_dot_array[drone_var_index,:] = np.array([msg.xd_dot.x, msg.xd_dot.y, msg.xd_dot.z])
        xd_ddot_array[drone_var_index,:] = np.array([msg.xd_ddot.x, msg.xd_ddot.y, msg.xd_ddot.z])
        x_v_array[drone_var_index,:] = np.array([msg.x_v.x, msg.x_v.y, msg.x_v.z])
        IMU_array[drone_var_index,:] = np.array([msg.IMU.x, msg.IMU.y, msg.IMU.z])
        rpy_array[drone_var_index,:] = np.array([msg.rpy.x, msg.rpy.y, msg.rpy.z])
        v_v_array[drone_var_index,:] = np.array([msg.v_v.x, msg.v_v.y, msg.v_v.z])
        ex_array[drone_var_index,:] = np.array([msg.ex.x, msg.ex.y, msg.ex.z])
        ev_array[drone_var_index,:] = np.array([msg.ev.x, msg.ev.y, msg.ev.z])
        eR_array[drone_var_index,:] = np.array([msg.eR.x, msg.eR.y, msg.eR.z])
        eW_array[drone_var_index,:] = np.array([msg.eW.x, msg.eW.y, msg.eW.z])
        f_array[drone_var_index,:] = np.array([msg.force])
        f_motor_array[drone_var_index,:] = np.array([msg.f_motor[0], msg.f_motor[1], msg.f_motor[2], msg.f_motor[3]])
        thr_array[drone_var_index,:] = np.array([msg.throttle[0], msg.throttle[1], msg.throttle[2], msg.throttle[3]])
        M_array[drone_var_index,:] = np.array([msg.Moment.x, msg.Moment.y, msg.Moment.z])
        gainX_array[drone_var_index,:] = np.array([msg.gainX[0], msg.gainX[1], msg.gainX[2], msg.gainX[3]])
        gainR_array[drone_var_index,:] = np.array([msg.gainR[0], msg.gainR[1], msg.gainR[2], msg.gainR[3]])
        dt_vicon_array[drone_var_index,:] = np.array([msg.dt_vicon_imu])

        drone_var_index += 1

    bag.close()

    return time_array, xd_array, xd_dot_array, xd_ddot_array, IMU_array, \
            rpy_array, x_v_array, v_v_array, ex_array, ev_array, eR_array, \
            eW_array, f_array, f_motor_array, thr_array, M_array, gainX_array, \
            gainR_array, dt_vicon_array



def plot_31_2(t, x, y, x_label, y_label, title):
    print('Plotting ' + title + ' ...')

    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    fig.suptitle(title, fontsize=12)

    plt.subplot(311)
    plt.plot(t[:], x[:,0],'b', label = x_label)
    plt.plot(t[:], y[:,0],'r', label = y_label)
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel('X')
    plt.legend()

    plt.subplot(312)
    plt.plot(t[:], x[:,1],'b', label = x_label)
    plt.plot(t[:], y[:,1],'r', label = y_label)
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel('Y')
    plt.legend()

    plt.subplot(313)
    plt.plot(t[:], x[:,2],'b', label = x_label)
    plt.plot(t[:], y[:,2],'r', label = y_label)
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel('Z')
    plt.legend()

    return



def plot_31_1(t, x, title, y_label_1, y_label_2, y_label_3):
    print('Plotting ' + title + ' ...')

    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    fig.suptitle(title, fontsize=12)

    plt.subplot(311)
    plt.plot(t[:], x[:,0],'b')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_1)

    plt.subplot(312)
    plt.plot(t[:], x[:,1],'b')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_2)

    plt.subplot(313)
    plt.plot(t[:], x[:,2],'b')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_3)

    return



def plot_41_1(t, x, title, y_label_1, y_label_2, y_label_3, y_label_4):
    print('Plotting ' + title + ' ...')

    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    fig.suptitle(title, fontsize=12)

    plt.subplot(411)
    plt.plot(t[:], x[:,0],'b')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_1)

    plt.subplot(412)
    plt.plot(t[:], x[:,1],'b')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_2)

    plt.subplot(413)
    plt.plot(t[:], x[:,2],'b')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_3)

    plt.subplot(414)
    plt.plot(t[:], x[:,3],'b')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_4)

    return



def plot_1(t, x, title, y_label):
    print('Plotting ' + title + ' ...')

    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    fig.suptitle(title, fontsize=12)

    plt.plot(t[:], x[:,0],'b')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label)

    return



def plot_trajectory(x_v_array):
    print('Plotting 3d trajectory ...')

    mpl.rcParams['legend.fontsize'] = 10

    fig_traj = plt.figure()
    ax = fig_traj.gca(projection='3d')

    ax.plot(x_v_array[:,0],x_v_array[:,1],x_v_array[:,2])
    ax.set_zlim(0, 3)

    return




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Parse a ROS Bag file and plot.')
    parser.add_argument('input_file', help='Input ROS Bag file')
    args = parser.parse_args()

    # read bag file and output variables
    time_array, xd_array, xd_dot_array, xd_ddot_array, IMU_array, rpy_array, \
        x_v_array, v_v_array, ex_array, ev_array, eR_array, eW_array, f_array, \
        f_motor_array, thr_array, M_array, gainX_array, gainR_array, \
        dt_vicon_array = read_bag_file(args.input_file)

    plot_trajectory(x_v_array)
    # plot_31_2(time_array, x_v_array, xd_array, 'vicon', 'desired', 'position')
    # plot_31_2(time_array, v_v_array, xd_dot_array, 'vicon', 'desired', 'velocity')
    # plot_31_1(time_array, IMU_array, 'IMU data', 'Phi', 'Theta', 'Psi')
    # plot_31_1(time_array, rpy_array, 'RPY data', 'Roll', 'Pitch', 'Yaw')
    # plot_31_1(time_array, ex_array, 'eX', 'X', 'Y', 'Z')
    # plot_31_1(time_array, ev_array, 'eV', 'V_x', 'V_y', 'V_z')
    # plot_31_1(time_array, eR_array, 'eR', 'X', 'Y', 'Z')
    # plot_31_1(time_array, eW_array, 'eW', 'X', 'Y', 'Z')
    # plot_41_1(time_array, f_motor_array, 'Motor Forces', 'Motor 1', 'Motor 2', 'Motor 3', 'Motor 4')
    # plot_41_1(time_array, thr_array, 'Throttle Values', 'Motor 1', 'Motor 2', 'Motor 3', 'Motor 4')
    # plot_41_1(time_array, gainR_array, 'Attitude Gains', 'kR', 'kW', 'kiR', 'cR')
    # plot_41_1(time_array, gainX_array, 'Position Gains', 'kX', 'kV', 'kiX', 'cX')
    # plot_1(time_array, f_array, 'Force', 'Force (N)')
    # plot_1(time_array, dt_vicon_array, 'Delta T from Vicon', 'dt')

    print('')
    print('Plotting completed!')

    plt.show()
