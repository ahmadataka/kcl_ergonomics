#!/usr/bin/env python
# The code is used to check whether the robot has done the task.

import roslib; roslib.load_manifest('kcl_ergonomics')
import math
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
class robot_speed(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('robot_speed_stat')
    # Define the ROS Subscriber used to read the robot's joint state
    joint_sub = rospy.Subscriber('/robot/joint_states', JointState, self.get_joint)
    # Define the ROS Publisher used to publish the status of whether the task has been finished or not
    self.flag_pub = rospy.Publisher('/fourbythree_topics/ergonomics/task_finish_flag', Int32, queue_size = 10)
    # Set the frequency to be 40 Hz
    r = rospy.Rate(40.0)
    # Wait
    r.sleep()
    # Initialize the speed
    self.speed = 0.0
    # Define the array used to send the status
    self.flag_sent = Int32()
    while not rospy.is_shutdown():
      # Check whether the speed is big
      if(self.speed>0.05):
          # Set the status
          self.flag_sent.data = 1
      else:
          # Otherwise, reset the status
          self.flag_sent.data = 0
      # Publish the status
      self.flag_pub.publish(self.flag_sent)
      # Sleep for 1/40 Hz
      r.sleep()

  # Used to read the robot's joint angle
  def get_joint(self,vec):
      # Calculate the joint's speed
      self.speed = (vec.velocity[4])**2+(vec.velocity[5])**2+(vec.velocity[2])**2+(vec.velocity[3])**2+(vec.velocity[6])**2+(vec.velocity[7])**2+(vec.velocity[8])**2


if __name__ == '__main__':
    try:
        robot_speed()
    except rospy.ROSInterruptException: pass
