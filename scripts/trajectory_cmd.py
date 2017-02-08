#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
import tf

def cmd():
    pub = rospy.Publisher('xd', PoseStamped, queue_size=10)
    rospy.init_node('d_pose', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    t_init = rospy.get_rostime()

    pose = PoseStamped()
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        pose.header.stamp = rospy.get_rostime()
        pose.pose.position.x = 2
        pose.pose.position.y = 1
        pose.pose.position.z = 1
        th = 0.5
        q = tf.transformations.quaternion_from_euler(0, 0, th)
        pose.pose.orientation = Quaternion(*q)

        rospy.get_time();
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd()
    except rospy.ROSInterruptException:
        pass
