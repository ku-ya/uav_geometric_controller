#!/usr/bin/env python
PACKAGE = 'odroid'
import roslib;roslib.load_manifest(PACKAGE)

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from rosserial_arduino.msg import Adc
# import csv
import time

# from dynamic_reconfigure.server import Server
# from calibration.cfg import TutorialsConfig

import dynamic_reconfigure.client


# filein = open('data.txt','w')
v_in = 0
cmd = 0
def throttle_callback():
    # Implement I2C command
    return
def callback(data):
    global v_in
    # rospy.loginfo("Command: %s, Voltage (0-1023): %s, Weight: %d g", cmd, data.adc0, int(data.adc0/1024.0*600))
    v_in = data.adc0/1024.0*600
    # filein.write(str(cmd)+','+str(v_in)+'\n')

# TODO: write a test sequence
# check for step responce
# Write to a file
# filein = open('data.txt','w')
# filein.write(str(data.adc0)+'\n')
# filein.close()


def callback_srv(config, level):
    rospy.loginfo("""Reconfigure Request: {double_cmd}, {on_off},\
          {str_param}, {bool_param}, {size}""".format(**config))
    return config

def run():
    rospy.init_node('DAQ', anonymous=False)
    rospy.Subscriber("/adc", Adc, callback)
    pub = rospy.Publisher("cmd", Bool, queue_size=10)
    rate = rospy.Rate(10)
    t_init = rospy.get_time()
    print 'here'
    rospy.wait_for_service("/calibration_config/parameter_descriptions")
    print 'this'
    client = dynamic_reconfigure.client.Client("/calibration_config", timeout=10, config_callback=callback_srv)
    loop_cout = 0
    while not rospy.is_shutdown():
        cmd_out = True
        # cmd_out = int((rospy.get_time()-t_init)*20.0) + 20
        rospy.Subscriber("/adc", Adc, callback)
        # filein.write(str(cmd)+','+str(v_in)+'\n')
        pub.publish(cmd_out)
        # client.update_configuration({"double_cmd":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})
        rate.sleep()

def linear_ramp():
    global v_in
    global cmd
    rospy.init_node('DAQ', anonymous=False)
    rospy.Subscriber("/adc", Adc, callback)
    pub = rospy.Publisher("cmd", Bool, queue_size=10)
    rate = rospy.Rate(10)
    t_init = rospy.get_time()

    loop_cout = 0
    while not rospy.is_shutdown():
        cmd_out = True
        if loop_cout > 2000:
            cmd_out = False
            for i in range(30):
                pub.publish(cmd_out)
                rate.sleep()
                rospy.loginfo("Loop #: %s, Bool: %s, V_out (0-1023): %s, Weight: %d g", loop_cout, cmd_out,v_in, int(v_in/1024.0*600))

            loop_cout = 0
            t_init = rospy.get_time()
        # cmd_out = int((rospy.get_time()-t_init)*20.0) + 20
        rospy.Subscriber("/adc", Adc, callback)
        rospy.loginfo("Loop #: %s, Bool: %s,Voltage (0-1023): %s, Weight: %d g", loop_cout, cmd_out,v_in, int(v_in/1024.0*600))
        # filein.write(str(cmd)+','+str(v_in)+'\n')
        pub.publish(cmd_out)
        loop_cout += 1
        rate.sleep()
    # filein.close()

if __name__ == '__main__':
    # rospy.init_node('DAQ', anonymous=True)
    # wfile = open('data.txt','w')
    # data_acq()
    # rospy.spin()
    # rospy.Subscriber("/adc", Adc, callback)
    # rospy.spin()
    run()
