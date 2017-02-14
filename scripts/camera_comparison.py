#!/usr/bin/env python
import roslib
roslib.load_manifest('odroid')
import sys
import rospy
import cv2
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf

def callback_laser(data):
  global t
  t = data.header.stamp
  t = rospy.Time.now()
  data.header.stamp = t
  # data.header.stamp = rospy.get_rostime()
  data.header.frame_id = 'world'
  pub = rospy.Publisher('/scan_c', LaserScan, queue_size=10)
  pub.publish(data)
  br_xtion = tf.TransformBroadcaster()
  br_xtion.sendTransform((0.3,0.1, 0),
                     tf.transformations.quaternion_from_euler(-np.pi/2, 0, -np.pi/2 + 7./180*np.pi),
                     t,
                     'camera_k',
                     'world')
  br = tf.TransformBroadcaster()
  br.sendTransform((0, 0, 0.1),
                     tf.transformations.quaternion_from_euler(-np.pi/2, 0, -np.pi/2 + 6./180*np.pi),
                     data.header.stamp,
                     'camera_c',
                     'world')
  pub.publish(data)
def callback_kinect(data):
    global t
    data.header.stamp = t
    data.header.frame_id = 'camera_c'
    pub = rospy.Publisher('/ptc_kinect', PointCloud2, queue_size=10)
    # br = tf.TransformBroadcaster()
    # br.sendTransform((0, 0, 0),
    #                  tf.transformations.quaternion_from_euler(np.pi/2, -np.pi/2 + 5./180*np.pi, 0),
    #                  data.header.stamp,
    #                  'world',
    #                  'camera_c')
    pub.publish(data)
def callback_xtion(data):
    global t
    data.header.stamp = t
    data.header.frame_id = 'camera_k'
    pub = rospy.Publisher('/ptc_xtion', PointCloud2, queue_size=10)
    pub.publish(data)
def listener():
  print('yo')
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

def main(args):
  # ic = image_converter()
  rospy.init_node('image_compare', anonymous=True)
  global t
  t = rospy.Time.now()
  rospy.Subscriber("/scan", LaserScan, callback_laser)
  rospy.Subscriber("/camera/depth/points", PointCloud2, callback_kinect)
  rospy.Subscriber("/xtion_camera/depth/points", PointCloud2, callback_xtion)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
