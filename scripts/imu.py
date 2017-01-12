#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import tf
import csv
import numpy as np

all_data = []

# all_data = []

def openfile():
    global all_data
    with open('system_data.txt', 'rb') as csvfile:
        data = csv.reader(csvfile, delimiter=',')
        for i, row in enumerate(data):
            all_data.append(row)


def readfile(line):
    global all_data
    if(line < len(all_data)):
        row = all_data[line]
    # for row in data:
        # print ', '.join(row)
        euler_angles = [row[i] for i in np.arange(3,0,-1)]
        f = [row[i] for i in range(17,23)]
        print euler_angles
        print f
    # delta_t = data{1};
    # euler_angles = [data{4}, data{3}, data{2}];
    # eR = [data{5}, data{6}, data{7}];
    # eW = [data{8}, data{9}, data{10}];
    # eI = [data{11}, data{12}, data{13}];
    # F = data{14};
    # M = [data{15}, data{16}, data{17}];
    # f = [data{18} data{19}, data{20}, data{21}, data{22}, data{23}];
    # thr = [data{24}, data{25}, data{26}, data{27}, data{28}, data{29}];
    # R = [data{30}, data{31}, data{32}, data{33}, data{34}, data{35}, data{36}, data{37}, data{38}];
    # Rdot = [data{39}, data{40}, data{41}, data{42}, data{43}, data{44}, data{45}, data{46}, data{47}];
    # W = [data{48}, data{49}, data{50}];

def talker():
    pub = rospy.Publisher('imu/imu', Imu, queue_size=10)
    imuMsg = Imu()
    # imuRawMsg = RazorImu()
    imuMsg.orientation_covariance = [999999 , 0 , 0,
    0, 9999999, 0,
    0, 0, 999999]
    imuMsg.angular_velocity_covariance = [9999, 0 , 0,
    0 , 99999, 0,
    0 , 0 , 0.02]
    imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0,
    0 , 0.2, 0,
    0 , 0 , 0.2]
    # Publish message
    imuMsg.linear_acceleration.x = 0.0#float(words[3]) # tripe axis accelerator meter
    imuMsg.linear_acceleration.y = 1.0#float(words[4])
    imuMsg.linear_acceleration.z = 2.0#float(words[5])

    imuMsg.angular_velocity.x = 3.0#float(words[9]) #gyro
    imuMsg.angular_velocity.y = 4.0#float(words[10])
    imuMsg.angular_velocity.z = 5.0#float(words[11])
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = 0.0 #q[0] #magnetometer
    imuMsg.orientation.y = 0.0#q[1]
    imuMsg.orientation.z = 0.0#q[2]
    imuMsg.orientation.w = 1.0#q[3]
    imuMsg.header.frame_id = 'base_link'


    rospy.init_node('fake_imu', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
        imuMsg.header.stamp= rospy.Time.now()
        # rospy.loginfo(hello_str)
        pub.publish(imuMsg)
        rate.sleep()

if __name__ == '__main__':
    # data = openfile()
    # readfile(1)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
