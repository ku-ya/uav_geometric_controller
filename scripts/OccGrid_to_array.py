#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import tf, sys
import csv
import numpy as np
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from math import floor

def callback(data):
    # tempolary hardcode the parameter
    x_lim = -10
    y_lim = -10
    x_width = 20
    y_width = 20
    resol = 0.1

    map_meta = data.info
    N = int(x_width/resol) + 1
    temp = np.ones((N,N))*50

    dimh = MultiArrayDimension()
    dimw = dimh
    dimh.size = data.info.height
    dimh.label = 'height'
    dimh.stride = 48
    dimw.size = data.info.width
    dimw.label = 'width'
    dimw.stride = 64

    datanp = np.array(data.data, dtype=float)
    datanp = datanp
    datanp[datanp<0] = 50
    for i in range(data.info.height):
        for j in range(data.info.width):
            idx_x = round(N/2-map_meta.height/2)#-data.info.origin.position.x/0.1)
            idx_y = round(N/2--data.info.origin.position.x/0.1)
            # temp[int(idx_x + i),int(idx_y + j)] = datanp[(i-1)*data.info.width+j]
    for i in range(80):
      for j in range(60):
        temp[int(N/2-40+i),int(N/2-30+j)] = 0.1

    map = OccupancyGrid()
    map.data = temp.flatten()
    map.info.width = int(N)
    map.info.height = int(N)
    map.info.resolution = 0.1
    map.info.origin = map_meta.origin
    # data.data
    pub = rospy.Publisher('/TheMap', Float64MultiArray,queue_size=10)
    pub_map = rospy.Publisher('/map_fixed', OccupancyGrid, queue_size=10)
    a = Float64MultiArray()
    a.layout.dim.append(dimh)
    a.layout.dim.append(dimw)
    a.layout.data_offset = 0
    a.data = temp.flatten()/100.
    pub_map.publish(map)
    pub.publish(a)
    br = tf.TransformBroadcaster()
    br.sendTransform((-map_meta.origin.position.x-10, -map_meta.origin.position.y-10, 0.0),
                 (0.0, 0.0, 0.0, 1.0),
                 rospy.Time.now(),
                 "map",
                 "world")
    # rospy.loginfo(rospy.get_caller_id() + "shape %s", a.data.shape)
def cmd_callback(msg):
    trajectory = msg.data
    N = len(trajectory)
    # print(N)
    n_seg = int(trajectory[1])
    # print(n_seg)
    t_step = trajectory[2:int(n_seg+3)]
    # print(t_step)
    dim = 2
    # n = floor((N-2-len(t_step))/(dim*n_seg)+0.5)
    # n = (N-2-len(t_step))/2/n_seg+1
    # print(n)
    trajectory = trajectory[int(n_seg+3):]

    a_x = trajectory[:len(trajectory)/2]
    a_y = trajectory[len(trajectory)/2:]
    # print(a_x)
    # print(a_y)

    print(len(a_x), len(a_y))

def map_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Map_listener', anonymous=True)

    rospy.Subscriber('/projected_map', OccupancyGrid, callback)
    rospy.Subscriber('DesTrajCoeffConstrPolyLeastSqrs2D', Float64MultiArray, cmd_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    map_listener()
