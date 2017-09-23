#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    th = 3.14/4
    q = tf.transformations.quaternion_from_euler(0, 0, th)
    msg = Quaternion(*q)

    imuMsg = Imu()
    imuMsg.header = data.header
    imuMsg.orientation = msg * data.orientation

    pub = rospy.Publisher('imu', Imu, queue_size=10)
    pub.publish(imuMsg)


def listener():
    rospy.init_node('imu2uav', anonymous=True)

    rospy.Subscriber("imu/imu", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
