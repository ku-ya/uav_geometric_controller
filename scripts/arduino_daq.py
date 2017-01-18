#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from rosserial_arduino.msg import Adc
import csv
import time


def throttle_callback():
    # Implement I2C command
    return
def callback(data):
    rospy.loginfo("Voltage: %s: Weight: %d g", data.adc0, data.adc0/1024.0*600)


# def data_acq():
#
#     rospy.Subscriber("/adc", Adc, callback)

# def save_value

# TODO: write a test sequence
# check for step responce

# Write to a file
# filein = open('data.txt','w')
# filein.write(str(data.adc0)+'\n')
# filein.close()
def linear_ramp():
    rospy.init_node('DAQ', anonymous=True)
    rospy.Subscriber("/adc", Adc, callback)
    pub = rospy.Publisher("cmd", Float64, queue_size=10)
    rate = rospy.Rate(10)
    t_init = rospy.get_time()
    cmd_out = 0.0
    while not rospy.is_shutdown():
        if cmd_out > 200:
            for i in range(30):
                pub.publish(0.0)
                rate.sleep()
            t_init = rospy.get_time()
        cmd_out = (rospy.get_time()-t_init)*20.0
        pub.publish(cmd_out)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('DAQ', anonymous=True)
    # wfile = open('data.txt','w')
    # data_acq()
    # rospy.spin()
    linear_ramp()
