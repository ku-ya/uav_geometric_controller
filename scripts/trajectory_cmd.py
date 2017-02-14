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
        'e': 'lissajous',
        'p': 'p2p',
        'r': 'reset',
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
        'e': 15,
        'p': 10,
        'h': 4,
    }
    return switcher.get(argument, "nothing")

def cmd(msg):
    mission = ''
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

        reset_height = 1.5
        min_flight_height = 0.2
        vel_up = 0.5
        vel_dn = -0.5
        a = 2*np.pi/t_total# maps t = 0:t_total to 0:2*pi
        b = 2*a# maps t = 0:t_total to 0:4*pi (Lissajous figure 8 double rotation in y)
        

        print mission

        t_init = (rospy.get_rostime()).to_sec()

        while not rospy.is_shutdown():
            # hello_str = "hello world %s" % rospy.get_time()
            cmd.header.stamp = rospy.get_rostime()
            # t_last = cmd.header.stamp.to_sec()

            x = y = z = x_dot = y_dot = z_dot = x_ddot = y_ddot = z_ddot = 0
            

            if mission=='lissajous':
                t_now = (rospy.get_rostime()).to_sec()
                t = t_now-t_init

                start_radians_x = np.pi/2
                start_radians_y = b/a*start_radians_x
                A = 2.0
                B = 1.0
                delta = np.pi/2
                x = A*np.sin(a*t+delta+start_radians_x)
                x_dot = a*A*np.cos(a*t*delta+start_radians_x)
                x_ddot = -a**2*A*np.sin(a*t*delta+start_radians_x)
                y = B*np.sin(b*t+start_radians_y)
                y_dot = b*B*np.cos(b*t+start_radians_y)
                y_ddot = -b**2*B*np.sin(b*t+start_radians_y)
                z = reset_height
                z_dot = 0.0
                z_ddot = 0.0

                if t > t_total:
                    print('Lissajous figure 8 complete')
                    break
            elif mission=='p2p':
                t_now = (rospy.get_rostime()).to_sec()
                t = t_now-t_init
                x = np.sin(a*t)
                y = np.sin(a*t)
                z = reset_height+np.sin(a*t)/2
                x_dot = np.cos(t)*a
                y_dot = np.cos(t)*a
                z_dot = (np.cos(t)/2)*a
                x_ddot = -np.sin(t)*a**2
                y_ddot = -np.sin(t)*a**2
                z_ddot = -(np.sin(t)/2)*a**2

                if t > t_total:
                    print('p2p complete')
                    break
            elif mission == 'takeoff':
                t_now = (rospy.get_rostime()).to_sec()
                t = t_now-t_init
                x = 0
                y = 0
                z = min_flight_height+vel_up*t if min_flight_height+vel_up*t < 1.5 else 1.5
                x_dot = 0
                y_dot = 0
                z_dot = vel_up
                if t > t_total:
                    print('takeoff complete')
                    break
            elif mission == 'reset':
                x = 0
                y = 0
                z = reset_height
                x_dot = 0
                y_dot = 0
                z_dot = 0
                x_ddot = 0
                y_ddot = 0
                z_ddot = 0

                print('resetting...')
                break
            elif mission == 'land':
                t_now = (rospy.get_rostime()).to_sec()
                t = t_now-t_init
                x = 0
                y = 0
                z = reset_height+t*vel_dn if reset_height+t*vel_dn > min_flight_height else 0
                x_dot = 0
                y_dot = 0
                z_dot = vel_dn
                x_ddot = 0
                y_ddot = 0
                z_ddot = 0
                if z <= min_flight_height:
                    rospy.set_param('/odroid_node/Motor', False)
                    pub.publish(cmd)
                    print('landing complete')
                    break

            cmd.xd.x = x
            cmd.xd.y = y
            cmd.xd.z = z

            cmd.xd_dot.x = x_dot
            cmd.xd_dot.y = y_dot
            cmd.xd_dot.z = z_dot

            cmd.xd_ddot.x = x_ddot
            cmd.xd_ddot.y = y_ddot
            cmd.xd_ddot.z = z_ddot

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
