#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import TwistStamped, Quaternion, AccelStamped
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np
from odroid.msg import trajectory_cmd

def char_to_mission(argument):
    switcher = {
        't': 'takeoff',
        'l': 'land',
        'f': 'figure8',
        'p': 'p2p',
        'h': 'hover',
        'm': 'motor',
        'w': 'warmup'
    }
    return switcher.get(argument, "nothing")

def char_to_speed(argument):
    switcher = {
        't': 4,
        'l': 4,
        'f': 15,
        'p': 10,
        'h': 4,
    }
    return switcher.get(argument, "nothing")

def cmd(msg):
    mission = ''
    speed = 0
    mission = char_to_mission(msg.data)
    t_total = char_to_speed(msg.data)

    pub = rospy.Publisher('xd', trajectory_cmd, queue_size=10)
    rate = rospy.Rate(100) # 10hz

    cmd = trajectory_cmd()
    cmd.header.frame_id = 'Quad'
    # rospy.set_param('/odroid_node/Motor', True)
    pub.publish(cmd)

    if mission == 'motor':
        if rospy.get_param('/odroid_node/Motor'):
            rospy.set_param('/odroid_node/Motor', False)
            print('Motor OFF')
        else:
            rospy.set_param('/odroid_node/Motor', True)
            print('Motor ON')
        rospy.set_param('/odroid_node/MotorWarmup', True)
        pub.publish(cmd)
    elif mission == 'warmup':
        if rospy.get_param('/odroid_node/MotorWarmup'):
            rospy.set_param('/odroid_node/MotorWarmup', False)
            print('Motor warmup OFF')
        else:
            rospy.set_param('/odroid_node/MotorWarmup', True)
            print('Motor warmup ON')
        pub.publish(cmd)

    else:
        if not rospy.get_param('/odroid_node/Motor'):
            rospy.set_param('/odroid_node/Motor', True)
            rospy.set_param('/odroid_node/MotorWarmup', True)
            print('Motor warmup ON')
            rospy.sleep(3)
            rospy.set_param('/odroid_node/MotorWarmup', False)

        dt = 0.01
        N = round(t_total / dt)
        pi_speed = np.pi * 2 / N
        speed = 1./N

        print mission

        t_init = (rospy.get_rostime()).to_sec()
        loop = 0.0

        while not rospy.is_shutdown():
            # hello_str = "hello world %s" % rospy.get_time()
            cmd.header.stamp = rospy.get_rostime()
            # t_last = cmd.header.stamp.to_sec()

            # t = (t_last - t_init)*speed
            x = y = z = x_dot = y_dot = z_dot = 0
            t = speed * loop

            if mission=='figure8':
                t = pi_speed * loop + np.pi/2
                a = 2.0
                x = a*np.cos(t)/(1+np.sin(t)**2)
                y = a*np.sin(t)*np.cos(t)/(1+np.sin(t)**2)
                z = 1.5 # 2 + np.sin(t*np.pi/2)
                x_dot = - a*np.sin(t)/(np.sin(t)**2 + 1) - 2*a*np.sin(t)*np.cos(t)**2/(np.sin(t)**2+1)**2
                y_dot = - a*np.sin(t)**2/(np.sin(t)**2 + 1) + a*np.cos(t)**2/(np.sin(t)**2+1) - 2*a*np.sin(t)**2*np.cos(t)**2/(np.sin(t)**2+1)**2
                z_dot = 0
                if loop > N:
                    print('figure8 complete')
                    break
            elif mission=='p2p':
                t = pi_speed * loop
                x = np.sin(t)
                y = np.sin(t)
                z = 1.5 + np.sin(t)/2
                x_dot = np.cos(t)
                y_dot = np.cos(t)
                z_dot = np.cos(t)/2
                if loop > N:
                    print('p2p complete')
                    break
            elif mission == 'takeoff':
                x = 0
                y = 0
                z = 1.5*speed*loop if 1.5*speed*loop < 1.5 else 1.5
                x_dot = 0
                y_dot = 0
                z_dot = 0
                if loop > N:
                    print('takeoff complete')
                    break
            elif mission == 'hover':
                x = 0
                y = 0
                z = 1.5
                x_dot = 0
                y_dot = 0
                z_dot = 0
                if loop > N:
                    print('hover complete')
                    break
            elif mission == 'land':
                x = 0
                y = 0
                z = 1.5 - loop/300. if 1.5 - loop/300. > 0.2 else 0
                x_dot = 0
                y_dot = 0
                z_dot = 0
                if z <= 0.2:
                    rospy.set_param('/odroid_node/Motor', False)
                    pub.publish(cmd)
                    print('landing complete')
                    break

            cmd.xd.x = x
            cmd.xd.y = y
            cmd.xd.z = z

            cmd.xd_dot.x = speed * x_dot
            cmd.xd_dot.y = speed * y_dot
            cmd.xd_dot.z = speed * z_dot

            loop = loop + 1.0
            pub.publish(cmd)
            rate.sleep()


def get_key():
    rospy.init_node('d_pose', anonymous=True)
    rospy.Subscriber('cmd_key', String, cmd)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.set_param('/controller/mode', 1)
        get_key()
    except rospy.ROSInterruptException:
        pass
