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
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import numpy as np
from numpy import linalg
import tf

def order_move(index_count):
    if(index_count==0):
        index_out = 6
        # print "UA Lat"
    elif(index_count==1):
        index_out = 1
        # print "LA Lat"
    elif(index_count==2):
        index_out = 4
        # print "Wrist Lat"
    elif(index_count==3):
        # index_out = 5
        index_out = 3
        # print "Wrist Front"
    elif(index_count==4):
        # index_out = 3
        index_out = 2
        # print "UA Front"
    # elif(index_count==5):
        # index_out = 2
        # print "LA Front"
    return index_out


class ergonomic_pose(object):
  def __init__(self):
    rospy.init_node('ergonomic_pose')
    self.mode = rospy.get_param('/ergo_mode')
    self.calib_mode = rospy.get_param('/robot_mode')
    if(self.mode == 0):
        self.full = 1
        self.angle_bound = 0.0
    else:
        self.full = 0
        self.angle_bound = 10.0
    self.left_angle = [70.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.right_angle = Float64MultiArray()
    self.pose_sent = Pose()
    self.state = 0
    self.delta = 0.0
    self.time_bound = 1.0
    self.dist = [0.0, 0.0, 0.0]
    self.a = 0.075
    self.a2 = 0.125
    self.b = 0.10
    self.b2 = 0.2
    self.l = 0.075
    self.ee_pose = [0.0, 0.0, 0.0]
    self.target = [0.0, 0.0, 0.0]
    self.euler = [0.0, 0.0, 0.0]
    self.eul_target = [0.0, 0.0, 0.0]
    # self.save_euler = self.euler
    self.save_euler = [0.0, 0.0, 0.0]
    self.assign_target = 0
    self.save_flag = 0
    self.rula_bound = 2
    self.rula = 1
    self.upper = 1
    self.lower = 1
    self.wrist = 1
    self.counter = 0
    self.rula_ind = 1
    self.abduct_stat = 0
    self.lateral_stat = 0
    self.pedal_stat = 0
    self.task_stat = 0
    self.state_count = 0
    self.first_move = 0
    self.range_stat = 0
    left_sub = rospy.Subscriber('left_human_angle', Float64MultiArray, self.get_left)
    euler_sub = rospy.Subscriber('/euler_gripper', Float64MultiArray, self.get_euler)
    rula_sub = rospy.Subscriber('/rula_score', Float64MultiArray, self.get_rula)
    upper_sub = rospy.Subscriber('/upper_arm_score', Float64MultiArray, self.get_upper)
    lower_sub = rospy.Subscriber('/lower_arm_score', Float64MultiArray, self.get_lower)
    wrist_sub = rospy.Subscriber('/wrist_score', Float64MultiArray, self.get_wrist)
    wrist_twist_sub = rospy.Subscriber('/wrist_twist_score', Float64MultiArray, self.get_wrist_tw)
    # abduct_sub = rospy.Subscriber('/abduction_status', Int32MultiArray, self.get_abduct)
    abduct_sub = rospy.Subscriber('/upper_arm_corstat', Int32, self.get_abduct)
    finish_sub = rospy.Subscriber('/movement_finish', Int32, self.get_finish)
    range_sub = rospy.Subscriber('/out_of_range', Int32, self.get_range)
    lateral_sub = rospy.Subscriber('/lower_arm_stat', Int32, self.get_lateral)
    pedal_sub = rospy.Subscriber('/pedal_status', Int32MultiArray, self.get_pedal)
    task_sub = rospy.Subscriber('/task_finish_flag', Int32, self.get_task)
    #endeffec_sub = rospy.Subscriber('/baxter/pose/current', Pose, self.get_pose)
    self.pose_pub = rospy.Publisher('baxter_target', Pose, queue_size = 10)
    calib_sub = rospy.Subscriber('/arm_calibrate', Float64MultiArray, self.get_arm)

    r = rospy.Rate(40.0)
    while not rospy.is_shutdown():
    #   print self.a
    #   print self.b
    #   print self.pedal_stat
      self.calib_mode = rospy.get_param('/robot_mode')
    #   print self.delta
    #   if(self.delta>self.time_bound):
    #       print "MOVE"
      if(self.full==0):
          self.calculate_pose()
      else:
          if(self.calib_mode == 1):
              self.full_ergo()
          else:
              self.assign_target = 0
              self.state = 0
              self.delta = 0.0
              self.counter = 0

      #pose_pub.publish(self.pose_sent)
      r.sleep()
        #rospy.spin()

  def get_left(self,vec):
    self.left_angle[0] = vec.data[0]
    self.left_angle[1] = vec.data[1]
    self.left_angle[2] = vec.data[2]
    self.left_angle[3] = vec.data[3]
    self.left_angle[4] = vec.data[4]
    self.left_angle[5] = vec.data[5]

  def get_arm(self,vec):
    # if(self.calib_mode == 1):
    self.a = self.a2 = vec.data[0]
    self.b = self.b2 = vec.data[1]


  def get_euler(self,vec):
    # if(self.save_flag == 0):
    #     self.save_euler = vec.data
    self.euler[0] = vec.data[0]
    self.euler[1] = vec.data[1]
    self.euler[2] = vec.data[2]
    self.save_flag = 1

  def get_rula(self,vec):
    self.rula = vec.data[0]

  def get_upper(self,vec):
    self.upper = vec.data[0]

  def get_lower(self,vec):
    self.lower = vec.data[0]

  def get_wrist(self,vec):
    self.wrist = vec.data[0]

  def get_wrist_tw(self,vec):
    self.wrist_twist = vec.data[0]

  def get_pose(self,vec):
    self.ee_pose[0] = vec.position.x
    self.ee_pose[1] = vec.position.y
    self.ee_pose[2] = vec.position.z

  def get_lateral(self,vec):
    self.lateral_stat = vec.data

  def get_abduct(self,vec):
    # self.abduct_stat = vec.data[0]
    self.abduct_stat = vec.data

  def get_finish(self,vec):
    # self.abduct_stat = vec.data[0]
    self.finish_stat = vec.data

  def get_task(self,vec):
    # self.abduct_stat = vec.data[0]
    self.task_stat = vec.data

  def get_range(self,vec):
    # self.abduct_stat = vec.data[0]
    self.range_stat = vec.data

  def get_pedal(self,vec):
    self.pedal_stat = vec.data[0]

  def calculate_pose(self):
    self.pose_sent.position.x = self.target[0]
    self.pose_sent.position.y = self.target[1]
    self.pose_sent.position.z = self.target[2]
    # print self.assign_target


    if(self.state == 1):
      # Coronal lower arm
        self.state = 0
        self.delta = 0.0
        self.dist[0] = -self.b2*(1.0-math.cos((self.left_angle[5])*math.pi/180.0))
        self.dist[1] = -self.b2*math.sin((self.left_angle[5])*math.pi/180.0)
        self.dist[2] = 0.0
        self.target[0] = self.ee_pose[0]+self.dist[0]
        self.target[1] = self.ee_pose[1]+self.dist[1]
        self.target[2] = self.ee_pose[2]+self.dist[2]
        self.pose_sent.position.x = self.target[0]
        self.pose_sent.position.y = self.target[1]
        self.pose_sent.position.z = self.target[2]
        self.pose_sent.orientation.x = 0.0
        self.pose_sent.orientation.y = 0.0
        self.pose_sent.orientation.z = 0.0
        self.pose_sent.orientation.w = 0.0

        self.assign_target = 1
        print "assign lower coronal"
        self.pose_pub.publish(self.pose_sent)


    elif(self.state == 2):
      #  lower arm sagital
        self.state = 0
        self.delta = 0.0
        self.dist[0] = -self.b*(1.0-math.sin((self.left_angle[0])*math.pi/180.0))
        self.dist[1] = 0.0
        self.dist[2] = self.b*math.cos((self.left_angle[0])*math.pi/180.0)
        self.target[0] = self.ee_pose[0]+self.dist[0]
        self.target[1] = self.ee_pose[1]+self.dist[1]
        self.target[2] = self.ee_pose[2]+self.dist[2]
        self.pose_sent.position.x = self.target[0]
        self.pose_sent.position.y = self.target[1]
        self.pose_sent.position.z = self.target[2]
        self.pose_sent.orientation.x = 0.0
        self.pose_sent.orientation.y = 0.0
        self.pose_sent.orientation.z = 0.0
        self.pose_sent.orientation.w = 0.0

        self.assign_target = 1
        print "assign lower sagital"
        self.pose_pub.publish(self.pose_sent)

    elif(self.state == 3):
      # Sagital Upper arm
        self.state = 0
        self.delta = 0.0
        self.dist[0] = self.a2*math.sin(self.left_angle[1]*math.pi/180.0)
        self.dist[1] = 0.0
        self.dist[2] = self.a2*(math.cos(self.left_angle[1]*math.pi/180.0)-1.0)

        self.target[0] = self.ee_pose[0]+self.dist[0]
        self.target[1] = self.ee_pose[1]+self.dist[1]
        self.target[2] = self.ee_pose[2]+self.dist[2]
        self.pose_sent.position.x = self.target[0]
        self.pose_sent.position.y = self.target[1]
        self.pose_sent.position.z = self.target[2]
        self.pose_sent.orientation.x = 0.0
        self.pose_sent.orientation.y = 0.0
        self.pose_sent.orientation.z = 0.0
        self.pose_sent.orientation.w = 0.0

        self.assign_target = 1
        print "assign upper sagital"
        self.pose_pub.publish(self.pose_sent)

    elif(self.state == 4):
        self.state = 0
        self.delta = 0.0
        for i in range(0,3):
            self.save_euler[i] = self.euler[i]
        self.eul_target[0] = self.euler[0]
        self.eul_target[1] = self.euler[1]
        self.eul_target[2] = self.euler[2]+(self.left_angle[2]*3.14/180.0)
        # self.eul_target[2] = (self.left_angle[2]*3.14/180.0)
        print "assign wrist"
        self.assign_target = 1
        quat_target = tf.transformations.quaternion_from_euler(self.eul_target[0],self.eul_target[1],self.eul_target[2])
        self.pose_sent.orientation.x = quat_target[0]
        self.pose_sent.orientation.y = quat_target[1]
        self.pose_sent.orientation.z = quat_target[2]
        self.pose_sent.orientation.w = quat_target[3]
        self.pose_sent.position.x = 0.0
        self.pose_sent.position.y = 0.0
        self.pose_sent.position.z = 0.0
        self.pose_pub.publish(self.pose_sent)

    elif(self.state == 5):
      # Wrist twist
        self.state = 0
        self.delta = 0.0
        self.dist[0] = -abs(self.a2*math.tan(self.left_angle[3]*math.pi/180.0/2)*math.sin(self.left_angle[3]*math.pi/180.0))
        self.dist[1] = 0.0
        if(self.left_angle[3]<0):
            self.dist[2] = -1*self.a2*math.sin(self.left_angle[3]*math.pi/180.0)
        elif(self.left_angle[3]>0):
            self.dist[2] = -self.a2*math.sin(self.left_angle[3]*math.pi/180.0)
        self.target[0] = self.ee_pose[0]+self.dist[0]
        self.target[1] = self.ee_pose[1]+self.dist[1]
        self.target[2] = self.ee_pose[2]+self.dist[2]
        self.pose_sent.position.x = self.target[0]
        self.pose_sent.position.y = self.target[1]
        self.pose_sent.position.z = self.target[2]
        self.pose_sent.orientation.x = 0.0
        self.pose_sent.orientation.y = 0.0
        self.pose_sent.orientation.z = 0.0
        self.pose_sent.orientation.w = 0.0

        self.assign_target = 1
        print "assign wrist up"
        self.pose_pub.publish(self.pose_sent)

    elif(self.state == 6):
      # coronal Upper arm
        self.state = 0
        self.delta = 0.0
        self.dist[0] = 0.0
        self.dist[1] = -self.a2*math.sin((self.left_angle[1])*math.pi/180.0)
        # self.dist[2] = -1*abs(self.a2*(math.tan(abs(self.left_angle[1])*math.pi/180.0/2.0))*math.sin(abs(self.left_angle[1])*math.pi/180.0))
        self.dist[2] = self.a2*(math.cos(self.left_angle[1]*math.pi/180.0)-1.0)
        self.target[0] = self.ee_pose[0]+self.dist[0]
        self.target[1] = self.ee_pose[1]+self.dist[1]
        self.target[2] = self.ee_pose[2]+self.dist[2]
        self.pose_sent.position.x = self.target[0]
        self.pose_sent.position.y = self.target[1]
        self.pose_sent.position.z = self.target[2]
        self.pose_sent.orientation.x = 0.0
        self.pose_sent.orientation.y = 0.0
        self.pose_sent.orientation.z = 0.0
        self.pose_sent.orientation.w = 0.0

        self.assign_target = 1
        print "assign upper coronal"
        self.pose_pub.publish(self.pose_sent)

  def full_ergo(self):
    #   print "mode"
    #   print self.mode
    #   print "rula_ind"
    #   print self.rula_ind
    #   print "assign"
    #   print self.assign_target

    #   print self.delta
    #   print "A"
      self.increase = 0
      if(self.rula>=self.rula_bound and self.task_stat == 0):
        #   print "B"
        #   if((self.first_move == 0 and self.task_stat==0) or (self.finish_stat==1 and self.range_stat==0) or (self.range_stat == 1 and self.task_stat==0)):
          if(self.task_stat==0):
            #   print self.finish_stat

              if(self.state_count == 0):
                self.begin = rospy.Time.now()
                self.state_count = 1
              self.current = rospy.Time.now()
              self.delta = float(self.current.secs)+float(self.current.nsecs)*(10.0**(-9.0)) - (float(self.begin.secs)+float(self.begin.nsecs)*(10.0**(-9.0)))
        #   print self.delta
        #   if(self.wrist>1)
        #   print "twist"
        #   Twist Correction
        #   self.mode = 4-self.counter

            #   print "lower"
          if(self.delta>self.time_bound):
              self.state = 0
              while(self.assign_target == 0):
                #   print "C"
                  self.increase = 0
                  self.mode = order_move(self.counter)

                #   self.mode = 2
                  if(self.mode==4 or self.mode==5):
                      self.rula_ind = self.wrist
                    #   print "wrist"
                  elif(self.mode==3 or self.mode==6):
                      self.rula_ind = self.upper
                    #   print "upper"
                  elif(self.mode==2 or self.mode==1):
                      self.rula_ind = self.lower
                  if(self.rula_ind > 1.0):
                    if(self.mode == 1):
                      if(self.lateral_stat==1 ):
                        # Lateral lower arm
                  	    self.state = 1
                      else:
                            self.increase = 1
                            self.target[0] = 0.0
                            self.target[1] = 0.0
                            self.target[2] = 0.0

                    elif(self.mode == 2):
                      if(self.lateral_stat==0):
                  	    self.state = 2
                      else:
                            self.increase = 1
                            self.target[0] = 0.0
                            self.target[1] = 0.0
                            self.target[2] = 0.0

                    elif(self.mode == 3):
                      if(self.abduct_stat==0):
                  	    self.state = 3
                      else:
                            self.increase = 1
                            self.target[0] = 0.0
                            self.target[1] = 0.0
                            self.target[2] = 0.0

                    elif(self.mode == 4):
                      self.state = 4

                    elif(self.mode == 5):
                      self.state = 5

                    elif(self.mode == 6):
                      if(self.abduct_stat!=0):
                        # Lateral upper arm
                  	    self.state = 6
                      else:
                        self.increase = 1
                        self.target[0] = 0.0
                        self.target[1] = 0.0
                        self.target[2] = 0.0
                    self.calculate_pose()
                    self.first_move = 1
                    #   self.pedal_stat = 0
                  else:
                      self.increase = 1

                  if(self.increase == 1):
                    if(self.counter!=4):
                        self.counter = self.counter+1
                    else:
                        self.counter = 0
              self.state_count = 0
            #   self.assign_target = 1
          if(self.assign_target == 1):
            #   print "move"
              self.assign_target = 0
              self.counter = 0
              self.pedal_stat = 0

      else:
          self.assign_target = 0
          self.state = 0
          self.delta = 0.0
          self.counter = 0
          self.state_count = 0
          #only upper arm
        #   self.counter = 2

        #   if(self.assign_target == 0):
            #   self.calculate_pose()
    #   if(self.rula>=self.rula_bound):
    #     #   print "upper"
    #     #   Upper arm Correction
    #       self.mode = 3
    #       while(self.assign_target == 0):
    #           self.calculate_pose()
    #   if(self.rula>=self.rula_bound):
    #     #   print "lower"
    #     #   Upper arm Correction
    #       self.mode = 2
    #       while(self.assign_target == 0):
    #           self.calculate_pose()




if __name__ == '__main__':
    try:
        ergonomic_pose()
    except rospy.ROSInterruptException: pass
