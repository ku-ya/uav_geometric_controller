#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import tf

def talker():
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    imuMsg = Imu()
    # imuRawMsg = RazorImu()
    imuMsg.orientation_covariance = [999999 , 0 , 0,
    0, 9999999, 0,
    0, 0, 999999]
    imuMsg.angular_velocity_covariance = [9999, 0 , 0,
    0 , 99999, 0,
    0 , 0 , 0.02]
    imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0,
    0 , 0.2, 0,
    0 , 0 , 0.2]
    # Publish message
    imuMsg.linear_acceleration.x = 0.0#float(words[3]) # tripe axis accelerator meter
    imuMsg.linear_acceleration.y = 1.0#float(words[4])
    imuMsg.linear_acceleration.z = 2.0#float(words[5])

    imuMsg.angular_velocity.x = 3.0#float(words[9]) #gyro
    imuMsg.angular_velocity.y = 4.0#float(words[10])
    imuMsg.angular_velocity.z = 5.0#float(words[11])
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = 0.0 #q[0] #magnetometer
    imuMsg.orientation.y = 0.0#q[1]
    imuMsg.orientation.z = 0.0#q[2]
    imuMsg.orientation.w = 1.0#q[3]
    imuMsg.header.frame_id = 'base_link'


    rospy.init_node('fake_imu', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
        imuMsg.header.stamp= rospy.Time.now()
        # rospy.loginfo(hello_str)
        pub.publish(imuMsg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
