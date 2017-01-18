#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from odroid.cfg import calibrationConfig

def callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\
    #       {str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("calibration_config", anonymous = False)
    srv = Server(calibrationConfig, callback)
    rospy.spin()
