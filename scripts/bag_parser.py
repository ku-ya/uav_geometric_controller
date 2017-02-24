#!/usr/bin/env python

import rosbag
from std_msgs.msg import Int32, String
import numpy as np
import pdb
import yaml
import argparse
import sys

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
    IMU_W_array = np.zeros((num_drone_var_msg,3))
    IMU_RPY_array = np.zeros((num_drone_var_msg,3))
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
    q_v_array = np.zeros((num_drone_var_msg,4))
    q_imu_array = np.zeros((num_drone_var_msg,4))


    drone_var_index = 0

    for topic, msg, t in bag.read_messages(topics=[drone_var_topic]):
        # pdb.set_trace()
        time_array[drone_var_index,0] = t.to_sec()
        xd_array[drone_var_index,:] = np.array([msg.xd.x, msg.xd.y, msg.xd.z])
        xd_dot_array[drone_var_index,:] = np.array([msg.xd_dot.x, msg.xd_dot.y, msg.xd_dot.z])
        xd_ddot_array[drone_var_index,:] = np.array([msg.xd_ddot.x, msg.xd_ddot.y, msg.xd_ddot.z])
        x_v_array[drone_var_index,:] = np.array([msg.x_v.x, msg.x_v.y, msg.x_v.z])
        IMU_W_array[drone_var_index,:] = np.array([msg.IMU.x, msg.IMU.y, msg.IMU.z])
        IMU_RPY_array[drone_var_index,:] = np.array([msg.rpy.x, msg.rpy.y, msg.rpy.z])
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
        q_v_array[drone_var_index,:] = np.array([msg.q_v.w, msg.q_v.x, msg.q_v.y, msg.q_v.z])
        q_imu_array[drone_var_index,:] = np.array([msg.q_imu.w, msg.q_imu.x, msg.q_imu.y, msg.q_imu.z])

        drone_var_index += 1

    bag.close()

    # shift the time array to the starting epoch
    time_array = time_array - time_array[0]

    return time_array, xd_array, xd_dot_array, xd_ddot_array, IMU_W_array, \
            IMU_RPY_array, x_v_array, v_v_array, ex_array, ev_array, eR_array, \
            eW_array, f_array, f_motor_array, thr_array, M_array, gainX_array, \
            gainR_array, dt_vicon_array, q_v_array, q_imu_array


def plot_31_2(t, x, y, x_label, y_label, title, latex):
    print('Plotting ' + title + ' ...')

    # for plotting normalized data


    mpl.rcParams['legend.fontsize'] = 10
    if latex:
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')

    fig, axarr = plt.subplots(3,1)
    fig.suptitle(title, fontsize=12)

    axarr[0].plot(t[:], x[:,0]/x[:,0].max(),'b', label = x_label)
    axarr[0].plot(t[:], y[:,0]/y[:,0].max(),'r', label = y_label)
    axarr[0].set_xlabel('Time (sec since epoch) ')
    if latex:
        axarr[0].set_ylabel(r'$X$')
    else:
        axarr[0].set_ylabel('X')
    axarr[0].legend()

    axarr[1].plot(t[:], x[:,1]/x[:,1].max(),'b', label = x_label)
    axarr[1].plot(t[:], y[:,1]/y[:,1].max(),'r', label = y_label)
    axarr[1].set_xlabel('Time (sec since epoch) ')
    if latex:
        axarr[1].set_ylabel(r'$Y$')
    else:
        axarr[1].set_ylabel('Y')
    axarr[1].legend()

    axarr[2].plot(t[:], x[:,2]/x[:,2].max(),'b', label = x_label)
    axarr[2].plot(t[:], y[:,2]/y[:,2].max(),'r', label = y_label)
    axarr[2].set_xlabel('Time (sec since epoch) ')
    if latex:
        axarr[2].set_ylabel(r'$Z$')
    else:
        axarr[2].set_ylabel('Z')
    axarr[2].legend()

    return


def plot_31_1(t, x, title, y_label_1, y_label_2, y_label_3, latex):
    print('Plotting ' + title + ' ...')

    mpl.rcParams['legend.fontsize'] = 10
    if latex:
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')

    fig, axarr = plt.subplots(3,1)
    fig.suptitle(title, fontsize=12)

    axarr[0].plot(t[:], x[:,0],'b')
    axarr[0].set_xlabel('Time (sec since epoch) ')
    axarr[0].set_ylabel(y_label_1)

    axarr[1].plot(t[:], x[:,1],'b')
    axarr[1].set_xlabel('Time (sec since epoch) ')
    axarr[1].set_ylabel(y_label_2)

    axarr[2].plot(t[:], x[:,2],'b')
    axarr[2].set_xlabel('Time (sec since epoch) ')
    axarr[2].set_ylabel(y_label_3)

    return


def plot_41_1(t, x, title, y_label_1, y_label_2, y_label_3, y_label_4, latex):
    print('Plotting ' + title + ' ...')

    mpl.rcParams['legend.fontsize'] = 10
    if latex:
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')

    fig, axarr = plt.subplots(4,1)
    fig.suptitle(title, fontsize=12)

    axarr[0].plot(t[:], x[:,0],'b')
    axarr[0].set_xlabel('Time (sec since epoch) ')
    axarr[0].set_ylabel(y_label_1)

    axarr[1].plot(t[:], x[:,1],'b')
    axarr[1].set_xlabel('Time (sec since epoch) ')
    axarr[1].set_ylabel(y_label_2)

    axarr[2].plot(t[:], x[:,2],'b')
    axarr[2].set_xlabel('Time (sec since epoch) ')
    axarr[2].set_ylabel(y_label_3)

    axarr[3].plot(t[:], x[:,3],'b')
    axarr[3].set_xlabel('Time (sec since epoch) ')
    axarr[3].set_ylabel(y_label_4)

    return


def plot_41_2(t, x, y, title, y_label_1, y_label_2, y_label_3, y_label_4, legend_1, legend_2, latex):
    print('Plotting ' + title + ' ...')

    mpl.rcParams['legend.fontsize'] = 10
    if latex:
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')

    fig = plt.figure()
    fig.suptitle(title, fontsize=12)

    plt.subplot(411)
    plt.plot(t[:], x[:,0],'b', label = legend_1)
    plt.plot(t[:], y[:,0],'r', label = legend_2)
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_1)
    plt.legend()

    plt.subplot(412)
    plt.plot(t[:], x[:,1],'b', label = legend_1)
    plt.plot(t[:], y[:,1],'r', label = legend_2)
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_2)

    plt.subplot(413)
    plt.plot(t[:], x[:,2],'b', label = legend_1)
    plt.plot(t[:], y[:,2],'r', label = legend_2)
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_3)

    plt.subplot(414)
    plt.plot(t[:], x[:,3],'b', label = legend_1)
    plt.plot(t[:], y[:,3],'r', label = legend_2)
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label_4)
    # plt.set_xlim(15, 35)


    return



def plot_1(t, x, title, y_label, latex):
    print('Plotting ' + title + ' ...')

    mpl.rcParams['legend.fontsize'] = 10
    if latex:
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')

    fig = plt.figure()
    fig.suptitle(title, fontsize=12)

    plt.plot(t[:], x[:,0],'b')
    plt.xlabel('Time (sec since epoch) ')
    plt.ylabel(y_label)

    return


def plot_trajectory(x_v_array, latex):
    print('Plotting 3d trajectory ...')

    mpl.rcParams['legend.fontsize'] = 10
    if latex:
        plt.rc('text', usetex=True)
        plt.rc('font', family='serif')

    fig_traj = plt.figure()
    ax = fig_traj.gca(projection='3d')

    ax.plot(x_v_array[:,0],x_v_array[:,1],x_v_array[:,2])
    if latex:
        ax.set_xlabel(r'$X$')
        ax.set_ylabel(r'$Y$')
        ax.set_zlabel(r'$Z$')
    else:
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
    #ax.set_zlim(0, 3)
    plt.axis('equal')

    return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Parse a ROS Bag file and plot.')
    parser.add_argument('input_file', help='Input ROS Bag file')
    parser.add_argument('-s', '--sensor', help='Plot sensor data - Vicon and IMU data.', action='store_true')
    parser.add_argument('-c', '--controller', help='Plot controller data - Gains and error varialbes.', action='store_true')
    parser.add_argument('-m', '--motor', help='Plot motor data - Throttle commands and forces.', action='store_true')
    parser.add_argument('-l', '--latex', help='Turn on LaTeX formatting and plot everything.', action='store_true')
    args = parser.parse_args()

    latex = args.latex

    try:
        # read bag file and output variables
        time_array, xd_array, xd_dot_array, xd_ddot_array, IMU_W_array, IMU_RPY_array, \
            x_v_array, v_v_array, ex_array, ev_array, eR_array, eW_array, f_array, \
            f_motor_array, thr_array, M_array, gainX_array, gainR_array, \
            dt_vicon_array, q_v_array, q_imu_array = read_bag_file(args.input_file)
    except (AttributeError):
        print("Error in reading the bag file. Most likely there was a change in the message format.")
        print("Skipping the plotting commands!")
        sys.exit()


    plot_trajectory(x_v_array, latex)
    # plot_41_1(time_array, thr_array, 'Throttle Values', 'Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', latex)
    # plot_31_2(time_array, IMU_W_array, x_v_array, 'IMU', 'Vicon', 'IMU + Vicon', latex)
    # plot_41_2(time_array, q_v_array, q_imu_array, 'Quaternion Comparison', 'w', 'q1', 'q2', 'q3', 'Vicon', 'IMU', latex)

    if args.latex:
        plot_31_2(time_array, x_v_array, xd_array, 'vicon', 'desired', 'position', latex)
        plot_31_2(time_array, v_v_array, xd_dot_array, 'vicon', 'desired', 'velocity', latex)
        plot_31_1(time_array, IMU_W_array, 'IMU data', r'$\Phi$', r'$\Theta$', r'$\Psi$', latex)
        plot_31_1(time_array, IMU_RPY_array, 'RPY data', 'Roll', 'Pitch', 'Yaw', latex)
        plot_1(time_array, dt_vicon_array, r'$\Delta T$ from Vicon', r'$dt$', latex)

        plot_31_1(time_array, ex_array, r'$e_X$', r'$X$', r'$Y$', r'$Z$', latex)
        plot_31_1(time_array, ev_array, r'$e_V$', r'$V_x$', r'$V_y$', r'$V_z$', latex)
        plot_31_1(time_array, eR_array, r'$eR$', r'$X$', r'$Y$', r'$Z$', latex)
        plot_31_1(time_array, eW_array, r'$eW$', r'$X$', r'$Y$', r'$Z$', latex)
        plot_41_1(time_array, gainR_array, 'Attitude Gains', r'$kR$', r'$kW$', r'$kiR$', r'$cR$', latex)
        plot_41_1(time_array, gainX_array, 'Position Gains', r'$kX$', r'$kV$', r'$kiX$', r'$cX$', latex)

        plot_41_1(time_array, f_motor_array, 'Motor Forces', 'Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', latex)
        plot_41_1(time_array, thr_array, 'Throttle Values', 'Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', latex)
        plot_1(time_array, f_array, 'Force', 'Force (N)', latex)

        args.sensor = False
        args.controller = False
        args.motor = False

    if args.sensor:
        plot_31_2(time_array, x_v_array, xd_array, 'vicon', 'desired', 'position', latex)
        plot_31_2(time_array, v_v_array, xd_dot_array, 'vicon', 'desired', 'velocity', latex)
        plot_31_1(time_array, IMU_W_array, 'IMU data', 'Phi', 'Theta', 'Psi', latex)
        plot_31_1(time_array, IMU_RPY_array, 'RPY data', 'Roll', 'Pitch', 'Yaw', latex)
        plot_1(time_array, dt_vicon_array, 'Delta T from Vicon', 'dt', latex)

    if args.controller:
        plot_31_1(time_array, ex_array, 'e_X', 'X', 'Y', 'Z', latex)
        plot_31_1(time_array, ev_array, 'e_V', 'V_x', 'V_y', 'V_z', latex)
        plot_31_1(time_array, eR_array, 'eR', 'X', 'Y', 'Z', latex)
        plot_31_1(time_array, eW_array, 'eW', 'X', 'Y', 'Z', latex)
        plot_41_1(time_array, gainR_array, 'Attitude Gains', 'kR', 'kW', 'kiR', 'cR', latex)
        plot_41_1(time_array, gainX_array, 'Position Gains', 'kX', 'kV', 'kiX', 'cX', latex)

    if args.motor:
        plot_41_1(time_array, f_motor_array, 'Motor Forces', 'Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', latex)
        plot_41_1(time_array, thr_array, 'Throttle Values', 'Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', latex)
        plot_1(time_array, f_array, 'Force', 'Force (N)', latex)



    print('')
    print('Plotting completed!')

    plt.show()
