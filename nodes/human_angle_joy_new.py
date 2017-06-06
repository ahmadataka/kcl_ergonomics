#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#


import roslib; roslib.load_manifest('kcl_ergonomics')
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class path_generator(object):
  def __init__(self):
    rospy.init_node('human_angle_joy')
    self.left_ang = [90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.right_ang = [90.0, 0.0, 0.0, 0.0]
    self.left_ang_sent = Float64MultiArray()
    self.right_ang_sent = Float64MultiArray()
    self.rec_vel = rospy.Subscriber('/fourbythree_topics/ergonomics/joy', Joy, self.ps3_callback)
    goal_pub = rospy.Publisher('/fourbythree_topics/ergonomics/left_human_angle', Float64MultiArray, queue_size = 10)
    goal2_pub = rospy.Publisher('right_human_angle', Float64MultiArray, queue_size = 10)
    calib_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/angle_calibrate', Float64MultiArray, self.get_calib)
    r = rospy.Rate(40)
    self.delta = 10.0
    self.wrist_init = [0.0, 0.0]
    self.theta_init = 0.0
    self.alpha_init = 0.0
    self.gamma_init = 0.0
    self.phi_init = 0.0
    while not rospy.is_shutdown():
        self.mode = rospy.get_param('/robot_mode')
        self.left_ang_sent.data = []
        if(self.mode == 0):
            self.left_ang_sent.data.append(self.left_ang[0])
            self.left_ang_sent.data.append(self.left_ang[1])
            self.left_ang_sent.data.append(self.left_ang[2])
            self.left_ang_sent.data.append(self.left_ang[3])
            self.left_ang_sent.data.append(self.left_ang[4])
            self.left_ang_sent.data.append(self.left_ang[5])
        else:
            self.left_ang_sent.data.append(90.0+self.left_ang[0]-self.theta_init)
            self.left_ang_sent.data.append(self.left_ang[1]-self.alpha_init)
            self.left_ang_sent.data.append(self.left_ang[2]+self.wrist_init[0])
            self.left_ang_sent.data.append(self.left_ang[3]-self.wrist_init[1])
            self.left_ang_sent.data.append(self.left_ang[4]-self.gamma_init)
            self.left_ang_sent.data.append(self.left_ang[5]-self.phi_init)
        self.right_ang_sent.data = []
        self.right_ang_sent.data.append(self.right_ang[0])
        self.right_ang_sent.data.append(self.right_ang[1])
        self.right_ang_sent.data.append(self.right_ang[2])
        self.right_ang_sent.data.append(self.right_ang[3])
	goal_pub.publish(self.left_ang_sent)
	goal2_pub.publish(self.right_ang_sent)
        r.sleep()

        #rospy.spin()
  def get_calib(self,vec):
    self.theta_init = vec.data[0]
    self.alpha_init = vec.data[1]
    self.gamma_init = vec.data[4]
    self.wrist_init[0] = -1*vec.data[2]
    self.wrist_init[1] = vec.data[3]
    self.phi_init = vec.data[5]

  def ps3_callback(self,msg):
    self.left_ang[0] += msg.axes[4]*self.delta
    self.left_ang[1] += msg.axes[5]*self.delta
    self.left_ang[2] += (msg.buttons[0]-msg.buttons[2])*self.delta
    self.left_ang[3] += (msg.buttons[1]-msg.buttons[3])*self.delta

if __name__ == '__main__':
    try:
        path_generator()
    except rospy.ROSInterruptException: pass
