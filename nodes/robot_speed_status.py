#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#


import roslib; roslib.load_manifest('kcl_ergonomics')
import math
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
class robot_speed(object):
  def __init__(self):
    rospy.init_node('robot_speed_stat')
    joint_sub = rospy.Subscriber('/robot/joint_states', JointState, self.get_joint)
    # self.rec_vel = rospy.Subscriber('/joy', Joy, self.ps3_callback)
    self.flag_pub = rospy.Publisher('/task_finish_flag', Int32, queue_size = 10)
    r = rospy.Rate(40.0)
    r.sleep()
    self.speed = 0.0
    self.flag_sent = Int32()
    while not rospy.is_shutdown():
      if(self.speed>0.05):
          self.flag_sent.data = 1
      else:
          self.flag_sent.data = 0
      self.flag_pub.publish(self.flag_sent)
      r.sleep()
    #   rospy.spin()

  def get_joint(self,vec):
      self.speed = (vec.velocity[4])**2+(vec.velocity[5])**2+(vec.velocity[2])**2+(vec.velocity[3])**2+(vec.velocity[6])**2+(vec.velocity[7])**2+(vec.velocity[8])**2
    #   print self.speed

  # def ps3_callback(self,msg):
  #   if(msg.axes[1]==1):
  #       self.flag_sent.data = 1
  #   elif(msg.axes[1]==-1)::
  #       self.flag_sent.data = 0
  #   self.flag_pub.publish(self.flag_sent)


if __name__ == '__main__':
    try:
        robot_speed()
    except rospy.ROSInterruptException: pass
