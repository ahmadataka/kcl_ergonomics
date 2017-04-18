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
from std_msgs.msg import Int32
import numpy as np
from numpy import linalg

class human_angle_node(object):
  def __init__(self):
    rospy.init_node('human_angle')
    self.mode = rospy.get_param('/robot_mode')
    self.wrist_on = rospy.get_param('~wrist_active')
    self.lhs = Pose()
    self.les = Pose()
    self.lts = Pose()
    self.lns = Pose()
    self.lrs = Pose()
    self.rhs = Pose()
    self.res = Pose()
    self.rts = Pose()
    self.rns = Pose()
    self.rhis = Pose()
    self.lhis = Pose()
    self.left_angle = Float64MultiArray()
    self.left_arm = Float64MultiArray()
    self.right_angle = Float64MultiArray()
    self.upper_arm_flag = Int32()
    self.wrist_angle = [0.0, 0.0]
    self.wrist_init = [0.0, 0.0]
    self.theta_init = 0.0
    self.alpha_init = 0.0
    self.gamma_init = 0.0
    self.phi_init = 0.0
    self.a = 0.0
    self.b = 0.0
    self.sagstat = 1
    self.corstat = 0
    left_hand_shoulder_sub = rospy.Subscriber('left_hand_shoulder', Pose, self.get_lhe)
    left_elbow_shou_sub = rospy.Subscriber('left_elbow_shoulder', Pose, self.get_les)
    left_neck_shoulder_sub = rospy.Subscriber('left_neck_shoulder', Pose, self.get_lns)
    left_torso_shou_sub = rospy.Subscriber('left_torso_shoulder', Pose, self.get_lts)
    left_right_shou_sub = rospy.Subscriber('left_right_shoulder', Pose, self.get_lrs)
    left_angle_pub = rospy.Publisher('left_human_angle', Float64MultiArray, queue_size = 10)
    right_hand_shoulder_sub = rospy.Subscriber('right_hand_shoulder', Pose, self.get_rhe)
    right_elbow_shou_sub = rospy.Subscriber('right_elbow_shoulder', Pose, self.get_res)
    right_neck_shoulder_sub = rospy.Subscriber('right_neck_shoulder', Pose, self.get_rns)
    right_torso_shou_sub = rospy.Subscriber('right_torso_shoulder', Pose, self.get_rts)
    right_hip_shou_sub = rospy.Subscriber('right_hip_shoulder', Pose, self.get_rhs)
    left_hip_shou_sub = rospy.Subscriber('left_hip_shoulder', Pose, self.get_lhs)
    right_angle_pub = rospy.Publisher('right_human_angle', Float64MultiArray, queue_size = 10)
    wrist_sub = rospy.Subscriber('wrist_euler', Float64MultiArray, self.get_wrist)
    calib_sub = rospy.Subscriber('angle_calibrate', Float64MultiArray, self.get_calib)
    sagstat_sub = rospy.Subscriber('upper_arm_sagstat', Int32, self.get_sagstat)
    corstat_sub = rospy.Subscriber('upper_arm_corstat', Int32, self.get_corstat)
    left_arm_pub = rospy.Publisher('left_arm', Float64MultiArray, queue_size = 10)
    # upper_arm_pub = rospy.Publisher('upper_arm_status', Int32, queue_size = 10)
    r = rospy.Rate(40.0)
    while not rospy.is_shutdown():
      self.calculate_angle()
      left_angle_pub.publish(self.left_angle)
      right_angle_pub.publish(self.right_angle)
      left_arm_pub.publish(self.left_arm)
    #   upper_arm_pub.publish(self.upper_arm_flag)
      r.sleep()
        #rospy.spin()

  def get_lhe(self,vec):
    self.lhs.position.x = vec.position.x
    self.lhs.position.y = vec.position.y
    self.lhs.position.z = vec.position.z

  def get_les(self,vec):
    self.les.position.x = vec.position.x
    self.les.position.y = vec.position.y
    self.les.position.z = vec.position.z

  def get_lns(self,vec):
    self.lns.position.x = vec.position.x
    self.lns.position.y = vec.position.y
    self.lns.position.z = vec.position.z

  def get_lts(self,vec):
    self.lts.position.x = vec.position.x
    self.lts.position.y = vec.position.y
    self.lts.position.z = vec.position.z

  def get_lrs(self,vec):
    self.lrs.position.x = vec.position.x
    self.lrs.position.y = vec.position.y
    self.lrs.position.z = vec.position.z

  def get_rhe(self,vec):
    self.rhs.position.x = vec.position.x
    self.rhs.position.y = vec.position.y
    self.rhs.position.z = vec.position.z

  def get_res(self,vec):
    self.res.position.x = vec.position.x
    self.res.position.y = vec.position.y
    self.res.position.z = vec.position.z

  def get_rns(self,vec):
    self.rns.position.x = vec.position.x
    self.rns.position.y = vec.position.y
    self.rns.position.z = vec.position.z

  def get_rts(self,vec):
    self.rts.position.x = vec.position.x
    self.rts.position.y = vec.position.y
    self.rts.position.z = vec.position.z

  def get_rhs(self,vec):
    self.rhis.position.x = vec.position.x
    self.rhis.position.y = vec.position.y
    self.rhis.position.z = vec.position.z

  def get_lhs(self,vec):
    self.lhis.position.x = vec.position.x
    self.lhis.position.y = vec.position.y
    self.lhis.position.z = vec.position.z

  def get_wrist(self,vec):
    self.mode = rospy.get_param('/robot_mode')
    if(self.wrist_on==1):
        if(self.mode == 0):
            self.wrist_angle[0] = -1*vec.data[0]*180.0/math.pi
            self.wrist_angle[1] = vec.data[2]*180.0/math.pi
        else:
            self.wrist_angle[0] = -1*vec.data[0]*180.0/math.pi+self.wrist_init[0]
            self.wrist_angle[1] = (vec.data[2]*180.0/math.pi-self.wrist_init[1])

  def get_calib(self,vec):
    self.theta_init = vec.data[0]
    self.alpha_init = vec.data[1]
    self.gamma_init = vec.data[4]
    self.wrist_init[0] = -1*vec.data[2]
    self.wrist_init[1] = vec.data[3]
    self.phi_init = vec.data[5]

  def get_sagstat(self,vec):
    self.sagstat = vec.data

  def get_corstat(self,vec):
    self.corstat = vec.data

  def calculate_angle(self):
    lhe_vec = np.matrix([[self.lhs.position.x-self.les.position.x], [self.lhs.position.y-self.les.position.y], [self.lhs.position.z-self.les.position.z]])
    les_vec = np.matrix([[self.les.position.x], [self.les.position.y], [self.les.position.z]])
    ltn_vec = np.matrix([[self.lts.position.x-self.lns.position.x], [self.lts.position.y-self.lns.position.y], [self.lts.position.z-self.lns.position.z]])
    lrs_vec = np.matrix([[self.lrs.position.x], [self.lrs.position.y], [self.lrs.position.z]])
    # rlh_vec = np.matrix([[self.rhis.position.x-self.lhis.position.x], [self.rhis.position.y-self.lhis.position.y], [self.rhis.position.z-self.lhis.position.z]])

    ltn_cross = np.matrix([[0.0, -ltn_vec[2,0], ltn_vec[1,0]], [ltn_vec[2,0], 0.0, -ltn_vec[0,0]],[-ltn_vec[1,0], ltn_vec[0,0], 0.0]])
    horizon = ltn_cross*les_vec

    sign = horizon.getT()*lrs_vec

    if((linalg.norm(lhe_vec)*linalg.norm(les_vec))!=0):
      cos_theta = (lhe_vec.getT()*les_vec)/(linalg.norm(lhe_vec)*linalg.norm(les_vec))
      left_theta = math.acos(cos_theta)*180.0/math.pi
    else:
      cos_theta = 1.0
      left_theta = 0.0

    if((linalg.norm(ltn_vec)*linalg.norm(les_vec))!=0):
      cos_alpha = (ltn_vec.getT()*les_vec)/(linalg.norm(ltn_vec)*linalg.norm(les_vec))
      left_alpha = math.acos(cos_alpha)*180.0/math.pi
    #   if(sign>0):
    #       left_alpha = -1*left_alpha
    #   else:
    #       left_alpha = left_alpha
    else:
      cos_alpha = 1.0
      left_alpha = 0.0


    # lsn_vec = np.matrix([[-1*self.lns.position.x], [-1*self.lns.position.y], [-1*self.lns.position.z]])
    # # a_vec = ltn_cross*les_vec
    # b_vec = ltn_cross*lsn_vec
    # if(linalg.norm(b_vec)):
    #     lesp_vec = les_vec - (((les_vec.getT()*b_vec)[0,0])/linalg.norm(b_vec))*(b_vec/linalg.norm(b_vec))
    #     # a_vec = ltn_cross*lesp_vec
    #     # self.gamma_flag = (a_vec.getT()*b_vec)
    #     # if(self.gamma_flag >= 0):
    #     #     self.upper_arm_flag.data = 1
    #     # else:
    #     #     self.upper_arm_flag.data = -1
    #     self.gamma_flag = (lesp_vec.getT()*lrs_vec)
    #     if(self.gamma_flag >= 0):
    #         self.upper_arm_flag.data = -1
    #     else:
    #         self.upper_arm_flag.data = 1


    # if((linalg.norm(les_vec)*linalg.norm(lrs_vec))!=0):
    #   cos_gamma = (lrs_vec.getT()*les_vec)/(linalg.norm(lrs_vec)*linalg.norm(les_vec))
    #   left_gamma = math.acos(cos_gamma)*180.0/math.pi - 90.0
    # if((linalg.norm(les_vec)*linalg.norm(rlh_vec))!=0):
    #   cos_gamma = (rlh_vec.getT()*les_vec)/(linalg.norm(rlh_vec)*linalg.norm(les_vec))
    #   left_gamma = math.acos(cos_gamma)*180.0/math.pi - 90.0
    if((linalg.norm(les_vec)*linalg.norm(lrs_vec))!=0):
      cos_gamma = (lrs_vec.getT()*les_vec)/(linalg.norm(lrs_vec)*linalg.norm(les_vec))
      left_gamma = math.acos(cos_gamma)*180.0/math.pi - 90.0
    else:
      cos_gamma = 1.0
      left_gamma = -90.0

    if((linalg.norm(lhe_vec)*linalg.norm(lrs_vec))!=0):
      cos_phi = (-1*lrs_vec.getT()*lhe_vec)/(linalg.norm(lrs_vec)*linalg.norm(lhe_vec))
      left_phi =  90.0 - math.acos(cos_phi)*180.0/math.pi
    else:
      cos_phi = 1.0
      left_phi = 90.0

    rhe_vec = np.matrix([[self.rhs.position.x-self.res.position.x], [self.rhs.position.y-self.res.position.y], [self.rhs.position.z-self.res.position.z]])
    res_vec = np.matrix([[self.res.position.x], [self.res.position.y], [self.res.position.z]])
    rtn_vec = np.matrix([[self.rts.position.x-self.rns.position.x], [self.rts.position.y-self.rns.position.y], [self.rts.position.z-self.rns.position.z]])

    if((linalg.norm(rhe_vec)*linalg.norm(res_vec))!=0):
      cos_theta = (rhe_vec.getT()*res_vec)/(linalg.norm(rhe_vec)*linalg.norm(res_vec))
      right_theta = math.acos(cos_theta)*180.0/math.pi
    else:
      cos_theta = 1.0
      right_theta = 0.0

    if((linalg.norm(rtn_vec)*linalg.norm(res_vec))!=0):
      cos_alpha = (rtn_vec.getT()*res_vec)/(linalg.norm(rtn_vec)*linalg.norm(res_vec))
      right_alpha = math.acos(cos_alpha)*180.0/math.pi
    else:
      cos_alpha = 1.0
      right_alpha = 0.0

    self.a = linalg.norm(les_vec)
    self.b = linalg.norm(lhe_vec)
    self.left_arm.data = []
    self.left_arm.data.append(self.a)
    self.left_arm.data.append(self.b)

    self.mode = rospy.get_param('/robot_mode')
    self.left_angle.data = []

    # print(self.phi_init)
    # print(self.mode)
    if(self.mode==0):
        self.left_angle.data.append(left_theta)
        self.left_angle.data.append(left_alpha)
        self.left_angle.data.append(self.wrist_angle[0])
        self.left_angle.data.append(self.wrist_angle[1])
        self.left_angle.data.append(left_gamma)
        self.left_angle.data.append(left_phi)
    else:
        self.left_angle.data.append(90.0+left_theta-self.theta_init)
        if(self.corstat == 0):
            self.left_angle.data.append((left_alpha-self.alpha_init)*self.sagstat)
        else:
            self.left_angle.data.append((left_alpha-self.alpha_init)*self.corstat)
        self.left_angle.data.append(self.wrist_angle[0])
        self.left_angle.data.append(self.wrist_angle[1])
        self.left_angle.data.append(left_gamma-self.gamma_init)
        self.left_angle.data.append(left_phi-self.phi_init)
    self.right_angle.data = []
    self.right_angle.data.append(right_theta)
    self.right_angle.data.append(right_alpha)

if __name__ == '__main__':
    try:
        human_angle_node()
    except rospy.ROSInterruptException: pass
