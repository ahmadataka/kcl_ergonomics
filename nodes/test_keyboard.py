#!/usr/bin/env python
import roslib
roslib.load_manifest('kcl_ergonomics')
import rospy
import tf
import readchar
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('read_keyboard')
    key_pub = rospy.Publisher('keyboard_input', String, queue_size = 10)
    rate = rospy.Rate(40.0)
    flag = 0
    while (not rospy.is_shutdown() and flag==0):
	key_data = String()
	#print "gimme something, please?"
	#something = raw_input()
	#print "thanks for giving me " + something
	something = (readchar.readchar())
	if(something == ""):
	  print("close")
	  flag = 1

	#print(something)
	#print("aka")
	key_data.data = something

	key_pub.publish(key_data)

        rate.sleep()
