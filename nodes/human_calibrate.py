#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#


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
    rospy.init_node('human_calibrate')
    self.left_angle = Float64MultiArray()
    self.left_arm = Float64MultiArray()
    self.left_angle.data = [90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.left_arm.data = [0.0, 0.0]
    left_sub = rospy.Subscriber('left_human_angle', Float64MultiArray, self.get_left)
    left_arm = rospy.Subscriber('left_arm', Float64MultiArray, self.get_left_arm)
    self.calib_pub = rospy.Publisher('angle_calibrate', Float64MultiArray, queue_size = 10)
    self.calib_arm_pub = rospy.Publisher('arm_calibrate', Float64MultiArray, queue_size = 10)
    r = rospy.Rate(40.0)
    # r.sleep()
    # while not rospy.is_shutdown():

      #pose_pub.publish(self.pose_sent)
    #   r.sleep()
    rospy.spin()

  def get_left(self,vec):
    self.mode = rospy.get_param('/robot_mode')
    if(self.mode==0):
        self.left_angle.data[0] = vec.data[0]
        self.left_angle.data[1] = vec.data[1]
        self.left_angle.data[2] = vec.data[2]
        self.left_angle.data[3] = vec.data[3]
        self.left_angle.data[4] = vec.data[4]
        self.left_angle.data[5] = vec.data[5]
        self.calib_pub.publish(self.left_angle)

  def get_left_arm(self,vec):
    self.mode = rospy.get_param('/robot_mode')
    if(self.mode==0):
        self.left_arm.data[0] = vec.data[0]
        self.left_arm.data[1] = vec.data[1]
        self.calib_arm_pub.publish(self.left_arm)

if __name__ == '__main__':
    try:
        human_calibrate()
    except rospy.ROSInterruptException: pass
