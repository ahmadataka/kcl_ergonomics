#!/usr/bin/env python
# This code is used to read from the keyboard input
import roslib
roslib.load_manifest('kcl_ergonomics')
import rospy
import tf
import readchar
from std_msgs.msg import String


if __name__ == '__main__':
    # Initialize the ROS Node
    rospy.init_node('read_keyboard')
    # Define the ROS Publisher used to send the character
    key_pub = rospy.Publisher('/fourbythree_topics/ergonomics/keyboard_input', String, queue_size = 10)
    # Set the rate to be 40 Hz
    rate = rospy.Rate(40.0)
    # Reset the flag used to check the incorrect character
    flag = 0
    while (not rospy.is_shutdown() and flag==0):
        # Initialize the class String()
    	key_data = String()
        # Read from the Serial data
        something = (readchar.readchar())
        # Check whether the character is the correct one
    	if(something == ""):
    	  print("close")
          # Set the flag and close the loop
    	  flag = 1

        # Read the character
        key_data.data = something
        # Publish the character
    	key_pub.publish(key_data)
        # Sleep until 1/40 Hz
        rate.sleep()
