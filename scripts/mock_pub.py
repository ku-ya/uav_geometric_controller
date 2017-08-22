import rospy
from uav_geometric_controller.msg import trajectory, states
import numpy as np

def talker():
    pub = rospy.Publisher('uav_state', states, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    data = states()
    time = 0.01
    while not rospy.is_shutdown():
        data.eW.x = np.sin(10*time)
        pub.publish(data)
        time += 0.01
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
