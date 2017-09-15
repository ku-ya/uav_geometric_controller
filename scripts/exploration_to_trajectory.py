#!/usr/bin/env python
import rospy
from uav_geometric_controller.msg import states, trajectory
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from tf.transformations import quaternion_from_euler

desiredPoseTopic = ''
desiredTwistTopic = ''
desiredAccelTopic = ''
desiredTrajectory = 'Jetson/xc'

class exp_to_traj(object):

    def __init__(self):
        rospy.init_node('trajectory_echo', anonymous=True)
        rospy.Subscriber(desiredPoseTopic, PoseStamped, self.callback_pose)
        rospy.Subscriber(desiredTwistTopic, TwistStamped, self.callback_twist)
        rospy.Subscriber(desiredAccelTopic, AccelStamped, self.callback_accel)
        self.pub = rospy.Publisher(desiredTrajectory, trajetory, queue_size= 1)
        self.cmd = trajetory()
        self.cmd.b1 = [1, 0, 0]

    def callback_pose(self, msg):
        pos = msg.pose.position
        self.cmd.xc = [pos.x, pos.x, pos.x]
        # TODO: add b1
        euler_from_quaternion(msg.pose.orientation)
        pass

    def callback_twist(self, msg):
        pos = msg.twist.linear
        self.cmd.xc_dot = [pos.x, pos.x, pos.x]
        pass

    def callback_accel(self, msg):
        pos = msg.accel.linear
        self.cmd.xc_2dot = [pos.x, pos.x, pos.x]
        pass

    def publish(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            try:
                self.pub.publish(cmd)
                rate.sleep()
            except (rospy.exceptions):
                pass

if __name__ == '__main__':
    testobj = exp_to_traj()
    testobj.publish()
