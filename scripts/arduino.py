#!/usr/bin/env python
import serial
import rospy

from std_msgs.msg import Int16
from odroid.msg import DAQ

filein = open('data.txt','w')

def run():
    rospy.init_node('DAQ', anonymous=False)
    pub = rospy.Publisher("daq", DAQ, queue_size=10)
    rate = rospy.Rate(10)
    ser = serial.Serial('/dev/ttyACM0',baudrate=9600) #Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
    ser.flushInput()
    ser.flushOutput()
    msg = DAQ()
    while not rospy.is_shutdown():
        data_raw = ser.readline()
        #   print(data_raw)
        words = data_raw.split()
    #   print words
    #   print len(data_raw)
        if len(words)==4:
          msg.cmd = int(words[1])
          msg.volt_q = int(words[3])
          msg.volt_v = 3.3/1024*msg.volt_q
          msg.grams = 200.0/372*msg.volt_q
          pub.publish(msg)
        filein.write(str(msg.cmd)+','+str(msg.volt_q)+'\n')
    #   rate.sleep()
    filein.close()

if __name__ == '__main__':
    run()
