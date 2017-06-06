#!/usr/bin/env python
# The code is used to change the parameter from the keyboard input.

import roslib; roslib.load_manifest('kcl_ergonomics')
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32

class param_from_key(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('parameter_set')
    self.mode_sent = Int32()
    self.mode_sent.data = rospy.get_param('/robot_mode')


    # Define the ROS Subscriber to read from keyboard or joystick
    self.rec_vel = rospy.Subscriber('/fourbythree_topics/ergonomics/joy', Joy, self.ps3_callback)
    self.mode_pub = rospy.Publisher('/fourbythree_topics/ergonomics/robot_mode', Int32, queue_size = 10)
    # Spin and wait for the subscribed data
    rospy.spin()

  # Function to read the keyboard or joystick data
  def ps3_callback(self,msg):
    # Check which keyboard / joystick key is pressed
    # Calibration mode: human needs to stand straight and calibrate the joint angle and arm's length
    # Reset mode: Robot goes back to initial pose
    # Keyboard 'z'

    if(msg.axes[2]==-1):
        # Used to exit the calibration mode and reset mode
        rospy.set_param("/robot_mode", 1)
        self.mode_sent.data = 1
        rospy.set_param("/reset_mode", 0)
    # Keyboard 'x'
    elif(msg.axes[2]==1):
        # Used to go to calibration mode
        rospy.set_param("/robot_mode", 0)
        self.mode_sent.data = 0
    # Keyboard 's'
    elif(msg.axes[1]==-1):
        # Used to go to reset mode
        rospy.set_param("/reset_mode", 1)
    self.mode_pub.publish(self.mode_sent)


if __name__ == '__main__':
    try:
        param_from_key()
    except rospy.ROSInterruptException: pass
