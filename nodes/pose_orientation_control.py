#!/usr/bin/python

# Used to
import rospy
import numpy as np
import PyKDL
import math
from baxter_pykdl import baxter_kinematics
from baxter_core_msgs.msg import JointCommand
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32

class cart_control(object):
  def __init__(self):
    #From torso to gripper
    rospy.init_node('pose_orientation_control')
    self.task_flag = 0
    self.mode = rospy.get_param('/robot_mode')
    self.jump = rospy.get_param('/jump_mode')
    self.control = rospy.get_param('/baxter_control')

    self.pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=10)
    self.pub_joint = rospy.Publisher('/desired_joint', Float64MultiArray, queue_size=10)
    self.pub_out = rospy.Publisher('/out_of_range', Int32, queue_size=10)
    self.pub_finish = rospy.Publisher('/movement_finish', Int32, queue_size=10)
    task_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/task_finish_flag', Int32, self.get_task)
    target_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/baxter_target', Pose, self.get_pose)
    joint_sub = rospy.Subscriber('/robot/joint_states', JointState, self.get_joint)
    pedal_sub = rospy.Subscriber('/pedal_status', Int32MultiArray, self.get_pedal)
    baxter_pose_sub = rospy.Subscriber('/baxter/pose/current', Pose, self.get_gripper)
    r = rospy.Rate(1000)
    self.kin = baxter_kinematics('left', 'wrist')
    self.num_joint = 7
    self.kin.print_kdl_chain()
    self.q_d = JointCommand()
    self.flag = 0
    self.joint_sent = Float64MultiArray()
    self.flag_out_sent = Int32()
    self.finish_sent = Int32()
    self.flag_not_move = 0
    self.begin = 0
    self.pedal_stat = 0
    self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #Initialization
    #self.init_baxter()

    #first trial
    #self.ee_pose = [0.912669, 0.375933, -0.251559]
    #self.ee_rot = [-0.03946, 0.731117, 0.0459021, 0.679562]

    #second trial
    #self.ee_pose = [0.960176, 0.690808, -0.0334902]
    #self.ee_rot = [-0.0476491, 0.658188, 0.0916147, 0.745738]

    #imad trial
    # self.ee_pose = [1.0991, 0.46266, 0.242346]
    # self.ee_pose = [0.7991, 0.56266, 0.242346]
    # self.ee_pose = [0.683, 0.691, 0.397]
    # self.ee_pose = [0.758, 0.704, 0.422]
    # self.ee_pose = [0.839, 0.295, 0.425]
    # self.ee_pose = [0.812, 0.254, 0.275]
    self.ee_pose = [0.807, 0.227, 0.195]
    self.pose_cur = [0.807, 0.227, 0.195]
    # self.ee_pose = [0.840, 0.173, 0.453]
    # self.ee_pose = [0.801, 0.1770, 0.4550]



    # self.ee_rot = [0.744612, -0.0118117, 0.666729, -0.0297645]
    # self.ee_rot = [0.7071, 0.0, 0.7071, 0.0]
    # self.ee_rot = [-0.008, 0.73, -0.0095, 0.682]
    # self.ee_rot = [0.663, -0.013, 0.746, -0.024]
    # self.ee_rot = [0.672, 0.037, 0.738, -0.017]
    # self.ee_rot = [0.630, 0.002, 0.776, 0.014]
    self.ee_rot = [0.641, 0.006, 0.767, 0.011]
    self.rot_cur = [0.641, 0.006, 0.767, 0.011]
    # self.ee_rot = [0.649, -0.001, 0.760, 0.002]
    # self.ee_rot = [0.650, 0.0290, 0.753, -0.034]


    self.joint_desired = self.kin.inverse_kinematics(self.ee_pose, self.ee_rot)
    # print self.joint_desired
    # print self.kin.forward_position_kinematics()
    # print self.mode
    self.time_start = rospy.get_rostime()
    self.trailing_edge = 0

    while not rospy.is_shutdown():
      #if(self.flag==1):
	#self.joint_desired = self.kin.inverse_kinematics(self.ee_pose, self.ee_rot)
      self.mode = rospy.get_param('/robot_mode')
      self.reset = rospy.get_param('/reset_mode')
      self.finish_sent.data = self.trailing_edge
      self.pub_finish.publish(self.finish_sent)
      self.flag_out_sent.data = self.flag_not_move
      self.pub_out.publish(self.flag_out_sent)
      if(self.mode == 0 or self.reset == 1):
        self.flag_not_move = 0
        self.init_baxter()
        if(self.trailing_edge == 1):
                print "ATAKA"
                # self.ee_pose = [0.807, 0.227, 0.195]
                # self.ee_rot = [0.641, 0.006, 0.767, 0.011]
                # for i in range(0,3):
                #     self.ee_pose[i] = self.pose_cur[i]
                #     self.ee_rot[i] = self.rot_cur[i]
        self.ee_pose = [0.807, 0.227, 0.195]
        self.ee_rot = [0.641, 0.006, 0.767, 0.011]
      else:
        # if(self.pedal_stat==0):
        if(self.trailing_edge == 1):
                print "ATAKA"
                for i in range(0,3):
                    self.ee_pose[i] = self.pose_cur[i]
                    self.ee_rot[i] = self.rot_cur[i]
        if(self.begin==0):
            self.constraint_avoidance()
            self.begin = 1
        # else:
            # self.joint_desired = self.cur_joint
    	self.q_d.mode = 1
    	self.q_d.names = []
    	self.q_d.command = []
    	self.q_d.names.append("left_s0");
    	self.q_d.names.append("left_s1");
    	self.q_d.names.append("left_e0");
    	self.q_d.names.append("left_e1");
    	self.q_d.names.append("left_w0");
    	self.q_d.names.append("left_w1");
    	self.q_d.names.append("left_w2");
        if(self.joint_desired != None and self.flag_inv == 1):
        	for i in range(0, self.num_joint):
        	  self.q_d.command.append(self.joint_desired[i])
    	  #self.q_d.command.append(0.0)
    	  #self.q_d.command.append(0.0)
    	  #if(i==5):
    	    #self.q_d.command.append(3.14/2)
    	  #else:
    	    #self.q_d.command.append(0.0)
    	#target = [0.2526144001579921, -0.8086621145177458, -0.3484868473528433, 1.6414301526683812, -2.2787972146691375, -0.2689820766219541, 2.429969726539231]
        if(self.control == 1):
            self.pub.publish(self.q_d)

  def get_pedal(self,vec):
    # self.pedal_stat = vec.data[0]
    self.pedal_stat = 0

  def get_task(self,vec):
    self.task_flag_before = self.task_flag
    self.task_flag = vec.data
    if(self.task_flag_before==1 and self.task_flag == 0):
        self.trailing_edge = 1
    else:
        self.trailing_edge = 0

  def get_gripper(self, end_eff):
    self.rot_cur[0] = end_eff.orientation.x
    self.rot_cur[1] = end_eff.orientation.y
    self.rot_cur[2] = end_eff.orientation.z
    self.rot_cur[3] = end_eff.orientation.w
    self.pose_cur[0] = end_eff.position.x
    self.pose_cur[1] = end_eff.position.y
    self.pose_cur[2] = end_eff.position.z

  def get_joint(self,vec):
      self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      self.cur_joint[0] = vec.position[4]
      self.cur_joint[1] = vec.position[5]
      self.cur_joint[2] = vec.position[2]
      self.cur_joint[3] = vec.position[3]
      self.cur_joint[4] = vec.position[6]
      self.cur_joint[5] = vec.position[7]
      self.cur_joint[6] = vec.position[8]


  def get_pose(self, end_eff):
    self.flag = 1
    # if(self.ergo_mode!=0):
    #     if(self.ergo_mode==4):
    #         self.ee_rot[0] = end_eff.orientation.x
    #         self.ee_rot[1] = end_eff.orientation.y
    #         self.ee_rot[2] = end_eff.orientation.z
    #         self.ee_rot[3] = end_eff.orientation.w
    #     else:
    #         self.ee_pose[0] = self.ee_pose[0] + end_eff.position.x
    #         self.ee_pose[1] = self.ee_pose[1] + end_eff.position.y
    #         self.ee_pose[2] = self.ee_pose[2] + end_eff.position.z
    # else:
    if(end_eff.position.x == 0 and end_eff.position.y == 0 and end_eff.position.z == 0):
        self.ee_rot[0] = end_eff.orientation.x
        self.ee_rot[1] = end_eff.orientation.y
        self.ee_rot[2] = end_eff.orientation.z
        self.ee_rot[3] = end_eff.orientation.w
    else:
        self.ee_pose[0] = self.ee_pose[0] + end_eff.position.x
        self.ee_pose[1] = self.ee_pose[1] + end_eff.position.y
        self.ee_pose[2] = self.ee_pose[2] + end_eff.position.z
    # self.ee_pose[0] = self.ee_pose[0] + end_eff.position.x
    # self.ee_pose[1] = self.ee_pose[1] + end_eff.position.y
    # self.ee_pose[2] = self.ee_pose[2] + end_eff.position.z
    # print "target_pose"
    # print self.ee_pose
    # print "target_quat"
    # print self.ee_rot
    # self.joint_desired = self.kin.inverse_kinematics(self.ee_pose, self.ee_rot)
    self.constraint_avoidance()


  def init_baxter(self):
    self.time_current = rospy.get_rostime()
    dur = self.time_current.secs - self.time_start.secs
    if(dur<5 and self.jump == 1):
        # self.joint_desired = [0.192591856848229, 1.0470000931980925, -0.016199539545360686, 0.4974881149430006, -0.17878449173465505, 0.02147115950051859, -0.013438517894237556]
        # self.joint_desired = [0.9756117799804688, 0.07516505852050782, -1.6678206097229005, 1.7732817887695314, -1.4545972804504395, -0.16451943931274415, 3.0564567163696292]
        self.joint_desired = [0.8433059371765137, 0.13575729957275393, -1.6908303215148928, 1.6413594411621095, -3.0457188508666992, 0.016490293450927736, 1.6624516769714357]
        # self.joint_desired = [0.21667478604125978, 0.06250971703491211, -1.6394419651794434, 2.01258279140625,  -3.0457188508666992, 1.0898933485473634, 1.589587589630127]

        self.q_d.mode = 1
        self.q_d.names = []
        self.q_d.command = []
        self.q_d.names.append("left_s0");
        self.q_d.names.append("left_s1");
        self.q_d.names.append("left_e0");
        self.q_d.names.append("left_e1");
        self.q_d.names.append("left_w0");
        self.q_d.names.append("left_w1");
        self.q_d.names.append("left_w2");
        for i in range(0, self.num_joint):
          self.q_d.command.append(self.joint_desired[i])
        self.pub.publish(self.q_d)

    else:
        # self.joint_desired = [0.2841699406311035, 0.6465729013549805, -1.4756895162597656, 1.9542915215332033,  -2.3546605067138673, 0.8467573939453126, 1.5723303057861329]
        self.joint_desired = [0.28647091181030276, 0.8440729275695802, -1.415864265600586, 1.8465293713073732,  -2.1337672735107422, 0.7938350568237306, 1.5151895215026856]
        # self.joint_desired = [0.016106798254394532, -0.11159710219116212,-1.5715633153930666, 1.9830536612731935,  -3.0476363268493656, 1.1957380227905274, 1.4599662132019045]
        # self.joint_desired = [-0.0122718462890625, 0.04908738515625, -1.664752648150635, 2.088131345123291, -3.0062188456237795, 1.3955390201843263, 1.6586167250061037]


        self.q_d.mode = 1
        self.q_d.names = []
        self.q_d.command = []
        self.q_d.names.append("left_s0");
        self.q_d.names.append("left_s1");
        self.q_d.names.append("left_e0");
        self.q_d.names.append("left_e1");
        self.q_d.names.append("left_w0");
        self.q_d.names.append("left_w1");
        self.q_d.names.append("left_w2");
        for i in range(0, self.num_joint):
          self.q_d.command.append(self.joint_desired[i])
        self.pub.publish(self.q_d)


  def constraint_avoidance(self):
    #   taken from http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications
      s1_min = -2.147
      s1_max = 1.047
      e1_min = -0.05
      e1_max = 2.618
      w1_min = -1.5707
      w1_max = 2.094
      s0_min = -1.7016
      s0_max = 1.7016
      e0_min = -3.0541
      e0_max = 3.0541
      w0_min = -3.059
      w0_max = 3.059
      w2_min = -3.059
      w2_max = 3.059
      joint_min = [s0_min, s1_min, e0_min, e1_min, w0_min, w1_min, w2_min]
      joint_max = [s0_max, s1_max, e0_max, e1_max, w0_max, w1_max, w2_max]
      self.flag_inv = 0
    #   while(flag_inv==0):
      flag_none = 0
        #   while(flag_none==0):
        #       print "loop"
        #       self.joint_desired = self.kin.inverse_kinematics(self.ee_pose, self.ee_rot)
        #       if(self.joint_desired != None):
        #           flag_none = 1

      self.joint_desired = self.kin.inverse_kinematics(self.ee_pose, self.ee_rot)
      counter = 0
      if(self.joint_desired == None):
          print "out of range"
        #   print self.ee_pose
        #   print self.ee_rot
          self.flag_not_move = 1
        #   self.flag_out_sent.data = self.flag_not_move
        #   self.pub_out.publish(self.flag_out_sent)
          for i in range(0,3):
              self.ee_pose[i] = self.pose_cur[i]
              self.ee_rot[i] = self.rot_cur[i]
      else:
          self.flag_not_move = 0
          for i in range(0, self.num_joint):
              self.joint_desired[i] = self.joint_desired[i] % (2*math.pi)
              if(self.joint_desired[i] > math.pi):
                  self.joint_desired[i] = self.joint_desired[i] - 2*math.pi
              if(self.joint_desired[i]>=joint_min[i] and self.joint_desired[i]<=joint_max[i]):
                  counter = counter+1
          if(counter == self.num_joint):
              self.flag_inv = 1
              print "success"
            #   print self.joint_desired
          else:
              print "constraint"
              print counter
            #   print self.joint_desired

if __name__ == "__main__":
    try:
        cart_control()
    except rospy.ROSInterruptException: pass
