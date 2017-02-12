#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import TwistStamped, Quaternion, AccelStamped
from std_msgs.msg import Float32MultiArray
import tf
import numpy as np
from odroid.msg import trajectory_cmd

def cmd():
    pub = rospy.Publisher('xd', trajectory_cmd, queue_size=10)
    rospy.init_node('d_pose', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    t_init = (rospy.get_rostime()).to_sec()

    cmd = trajectory_cmd()
    cmd.header.frame_id = 'Quad'
    dt = 0.01

    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        cmd.header.stamp = rospy.get_rostime()
        t_last = cmd.header.stamp.to_sec()
        a = 2.0
        t = (t_last - t_init)*0.5

        cmd.xd.x = a*np.cos(t)/(1+np.sin(t)**2)
        cmd.xd.y = a*np.sin(t)*np.cos(t)/(1+np.sin(t)**2)
        cmd.xd.z = 2

        cmd.xd_dot.x = - a*np.sin(t)/(np.sin(t)**2 + 1) - 2*a*np.sin(t)*np.cos(t)**2/(np.sin(t)**2+1)**2
        cmd.xd_dot.y = - a*np.sin(t)**2/(np.sin(t)**2 + 1) + a*np.cos(t)**2/(np.sin(t)**2+1) - 2*a*np.sin(t)**2*np.cos(t)**2/(np.sin(t)**2+1)**2
        cmd.xd_dot.z = 0

        # theta = 1
        # q = tf.transformations.quaternion_from_euler(0, 0, theta)
        # pose.pose.orientation = Quaternion(*q)
        # print(pose)
        rospy.get_time();
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd()
    except rospy.ROSInterruptException:
        pass
