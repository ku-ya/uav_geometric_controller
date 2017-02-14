#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	pub = rospy.Publisher('cmd_key', String, queue_size = 1)
	rospy.init_node('keyboard')
	# speed = rospy.get_param("~speed", 0.5)
	# turn = rospy.get_param("~turn", 1.0)
	status = 0
	try:
		# print vels(speed,turn)
		while(1):
			key = getKey()
			print key
			if (key == '\x03'):
				break
			pub.publish(key)
	except:
		print e
	finally:
		# pub.publish(twist)
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
