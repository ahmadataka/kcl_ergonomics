#!/usr/bin/env python
# The code is used to generate the human's joint angle calibration offset.
# Created by King's College London and Queen Mary University of London, 2017.

import roslib; roslib.load_manifest('kcl_ergonomics')
import math
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import numpy as np
from numpy import linalg
import tf

class human_calibrate(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('human_calibrate')
    # Define the array to read the human's joint angle and arm length
    self.left_angle = Float64MultiArray()
    self.left_arm = Float64MultiArray()
    # Initialize the arrays
    self.left_angle.data = [90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.left_arm.data = [0.0, 0.0]

    # Define the ROS Subscriber used to read the human's joint angle and arm length
    left_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/left_human_angle', Float64MultiArray, self.get_left)
    left_arm = rospy.Subscriber('/fourbythree_topics/ergonomics/left_arm', Float64MultiArray, self.get_left_arm)
    # Define the ROS Publisher used to publish the human's joint angle and arm length offset
    self.calib_pub = rospy.Publisher('/fourbythree_topics/ergonomics/angle_calibrate', Float64MultiArray, queue_size = 10)
    self.calib_arm_pub = rospy.Publisher('/fourbythree_topics/ergonomics/arm_calibrate', Float64MultiArray, queue_size = 10)
    
    # Spin and receive the data from subscriber
    rospy.spin()

  # Used to read the human's joint angle
  def get_left(self,vec):
    # Check whether the human is in calibration mode
    self.mode = rospy.get_param('/robot_mode')
    if(self.mode==0):
        # Executed while in calibration mode
        # Read the current joint angle
        self.left_angle.data[0] = vec.data[0]
        self.left_angle.data[1] = vec.data[1]
        self.left_angle.data[2] = vec.data[2]
        self.left_angle.data[3] = vec.data[3]
        self.left_angle.data[4] = vec.data[4]
        self.left_angle.data[5] = vec.data[5]
        # Publish the current joint angle as an offset
        self.calib_pub.publish(self.left_angle)

  # Used to read the human's arm length
  def get_left_arm(self,vec):
    # Check whether the human is in calibration mode
    self.mode = rospy.get_param('/robot_mode')
    if(self.mode==0):
        # Executed while in calibration mode
        # Read the current arm length
        self.left_arm.data[0] = vec.data[0]
        self.left_arm.data[1] = vec.data[1]
        # Publish the current arm length as an offset
        self.calib_arm_pub.publish(self.left_arm)

if __name__ == '__main__':
    try:
        human_calibrate()
    except rospy.ROSInterruptException: pass
