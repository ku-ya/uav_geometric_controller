#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from rosserial_arduino.msg import Adc
import csv



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


if __name__ == '__main__':
    rospy.init_node('DAQ', anonymous=True)
    # wfile = open('data.txt','w')
    rospy.Subscriber("/adc", Adc, callback)
    # data_acq()
    rospy.spin()
