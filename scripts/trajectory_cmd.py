#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import TwistStamped, Quaternion
import tf
import numpy as np

def cmd():
    pub = rospy.Publisher('xd', TwistStamped, queue_size=10)
    rospy.init_node('d_pose', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    t_init = (rospy.get_rostime()).to_sec()

    xd = TwistStamped()
    xd.header.frame_id = 'Quad'
    dt = 0.01
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        xd.header.stamp = rospy.get_rostime()
        t_last = xd.header.stamp.to_sec()
        a = 2.0
        t = (t_last - t_init)*0.5
        pose.twist.linear.x = a*np.cos(t)/(1+np.sin(t)**2)
        pose.twist.linear.y = a*np.sin(t)*np.cos(t)/(1+np.sin(t)**2)
        pose.twist.linear.z = 2

        xd.twist.angular.x = - a*np.sin(t)/(np.sin(t)**2 + 1) - 2*a*np.sin(t)*np.cos(t)**2/(np.sin(t)**2+1)**2
        xd.twist.angular.y = - a*np.sin(t)**2/(np.sin(t)**2 + 1) + a*np.cos(t)**2/(np.sin(t)**2+1) - 2*a*np.sin(t)**2*np.cos(t)**2/(np.sin(t)**2+1)**2
        xd.twist.angular.z = 0
        # theta = 1
        # q = tf.transformations.quaternion_from_euler(0, 0, theta)
        # pose.pose.orientation = Quaternion(*q)
        # print(pose)
        rospy.get_time();
        pub.publish(xd)
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd()
    except rospy.ROSInterruptException:
        pass
