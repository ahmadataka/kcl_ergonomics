#!/usr/bin/env python
# The code is used to generate the human's joint angle.
# Created by King's College London and Queen Mary University of London, 2017.

import roslib; roslib.load_manifest('kcl_ergonomics')
import math
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32

class path_generator(object):
  def __init__(self):
    rospy.init_node('human_angle_joy')
    # This variable gets the parameter 'robot_mode' to check whether the human is in calibration mode
    self.mode = rospy.get_param('/robot_mode')
    # Initilize the human joint angle
    self.left_ang = [90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Initialize the message data
    self.left_ang_sent = Float64MultiArray()

    # ROS Subscriber used to read input from keyboard
    self.rec_vel = rospy.Subscriber('/fourbythree_topics/ergonomics/joy', Joy, self.ps3_callback)
    # ROS Publisher used to send the human joint angle value
    goal_pub = rospy.Publisher('/fourbythree_topics/ergonomics/left_human_angle', Float64MultiArray, queue_size = 10)

    # ROS Subscriber used to read the calibrated angle
    calib_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/angle_calibrate', Float64MultiArray, self.get_calib)
    # ROS Subscriber used to read the mode of the calibration
    mode_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/robot_mode', Int32, self.get_mode)
    # Setting the data rate as 40 Hz
    r = rospy.Rate(40)

    # Initialize all the joint angles
    self.delta = 10.0
    self.wrist_init = [0.0, 0.0]
    self.theta_init = 0.0
    self.alpha_init = 0.0
    self.gamma_init = 0.0
    self.phi_init = 0.0

    while not rospy.is_shutdown():
        # Clear the array
        self.left_ang_sent.data = []
        # Check whether the human is in calibration mode
        if(self.mode == 0):
            # If yes, simply send the angle values
            self.left_ang_sent.data.append(self.left_ang[0])
            self.left_ang_sent.data.append(self.left_ang[1])
            self.left_ang_sent.data.append(self.left_ang[2])
            self.left_ang_sent.data.append(self.left_ang[3])
            self.left_ang_sent.data.append(self.left_ang[4])
            self.left_ang_sent.data.append(self.left_ang[5])
        else:
            # If no, send the celibrated angle values
            self.left_ang_sent.data.append(90.0+self.left_ang[0]-self.theta_init)
            self.left_ang_sent.data.append(self.left_ang[1]-self.alpha_init)
            self.left_ang_sent.data.append(self.left_ang[2]+self.wrist_init[0])
            self.left_ang_sent.data.append(self.left_ang[3]-self.wrist_init[1])
            self.left_ang_sent.data.append(self.left_ang[4]-self.gamma_init)
            self.left_ang_sent.data.append(self.left_ang[5]-self.phi_init)
        # Publish the message
        goal_pub.publish(self.left_ang_sent)
        # Sleep until 1/40 s
        r.sleep()

  def get_mode(self,vec):
      # Check whether the human is in calibration mode
      self.mode = vec.data

  def get_calib(self,vec):
    # Get the calibrated angle value
    self.theta_init = vec.data[0]
    self.alpha_init = vec.data[1]
    self.gamma_init = vec.data[4]
    self.wrist_init[0] = -1*vec.data[2]
    self.wrist_init[1] = vec.data[3]
    self.phi_init = vec.data[5]

  def ps3_callback(self,msg):
    # Get the input from keyboard
    self.left_ang[0] += msg.axes[4]*self.delta
    self.left_ang[1] += msg.axes[5]*self.delta
    self.left_ang[2] += (msg.buttons[0]-msg.buttons[2])*self.delta
    self.left_ang[3] += (msg.buttons[1]-msg.buttons[3])*self.delta

if __name__ == '__main__':
    try:
        path_generator()
    except rospy.ROSInterruptException: pass
