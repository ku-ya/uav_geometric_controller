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

    pose = TwistStamped()
    dt = 0.01
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        pose.header.stamp = rospy.get_rostime()
        t_last = pose.header.stamp.to_sec()
        a = 2.0
        t = (t_last - t_init)*0.5
        pose.twist.linear.x = a*np.cos(t)/(1+np.sin(t)**2)
        pose.twist.linear.y = a*np.sin(t)*np.cos(t)/(1+np.sin(t)**2)
        pose.twist.linear.z = 2

        pose.twist.angular.x = 0#(- a*np.sin(t)/(np.sin(t)**2 + 1) - 2*a*np.sin(t)*np.cos(t)**2/(np.sin(t)**2+1)**2)*dt
        pose.twist.angular.y = 0#(- a*np.sin(t)**2/(np.sin(t)**2 + 1) + a*np.cos(t)**2/(np.sin(t)**2+1) - 2*a*np.sin(t)**2*np.cos(t)**2/(np.sin(t)**2+1)**2)*dt
        pose.twist.angular.z = 0

        # th = 1
        # q = tf.transformations.quaternion_from_euler(0, 0, th)
        # pose.pose.orientation = Quaternion(*q)
        print(pose)
        rospy.get_time();
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd()
    except rospy.ROSInterruptException:
        pass
