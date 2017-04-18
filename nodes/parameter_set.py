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
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32

class param_from_key(object):
  def __init__(self):
    rospy.init_node('parameter_set')
    self.pedal = Int32MultiArray()
    self.stat_pub = rospy.Publisher('pedal_status', Int32MultiArray, queue_size = 10)
    self.rec_vel = rospy.Subscriber('/joy', Joy, self.ps3_callback)
    # self.rec_pedal = rospy.Subscriber('/pedal_signal', Int32, self.get_pedal)
    r = rospy.Rate(40)
    # while not rospy.is_shutdown():

    # r.sleep()

    rospy.spin()

  def ps3_callback(self,msg):
    if(msg.axes[2]==-1):
        rospy.set_param("/robot_mode", 1)
        rospy.set_param("/reset_mode", 0)
    elif(msg.axes[2]==1):
        rospy.set_param("/robot_mode", 0)
    elif(msg.axes[1]==-1):
        rospy.set_param("/reset_mode", 1)
    if(msg.axes[3]==1):
        self.pedal.data = []
        self.pedal.data.append(1)
        self.stat_pub.publish(self.pedal)
    else:
        self.pedal.data = []
        self.pedal.data.append(0)
        self.stat_pub.publish(self.pedal)

  # def get_pedal(self,msg):


if __name__ == '__main__':
    try:
        param_from_key()
    except rospy.ROSInterruptException: pass
