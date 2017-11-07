#!/usr/bin/env python
# The code is used to generate the human's joint angle.
# Created by King's College London and Queen Mary University of London, 2017.
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
    # This variable gets the parameter 'robot_mode' to check whether the human is in calibration mode
    self.mode = rospy.get_param('/robot_mode')
    # This variable gets the parameter 'wrist_active' to check whether the wrist_sensor is used
    self.wrist_on = rospy.get_param('~wrist_active')

    # Initialize variables used to read the pose data from human's link
    self.left_hand_shoulder = Pose()
    self.left_elbow_shoulder = Pose()
    self.left_torso_shoulder = Pose()
    self.left_neck_shoulder = Pose()
    self.left_right_shoulder = Pose()

    # Initialize variables used to send human's joint angle message into a topic
    self.left_angle = Float64MultiArray()
    # Initialize variables used to send human's arm length message into a topic
    self.left_arm = Float64MultiArray()

    # Initialize wrist angle array
    self.wrist_angle = [0.0, 0.0]
    self.wrist_init = [0.0, 0.0]

    # Initialize human joint angle
    self.theta_init = 0.0
    self.alpha_init = 0.0
    self.gamma_init = 0.0
    self.phi_init = 0.0

    # Initialize human's arm length
    self.a = 0.0
    self.b = 0.0

    # Initialize sagital and coronal flag
    self.sagital_status = 1
    self.coronal_status = 0

    # Topics subscriber initialization.
    # Subscribe to human's skeleton position
    left_hand_shoulder_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/left_hand_shoulder', Pose, self.get_hand_elbow)
    left_elbow_shou_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/left_elbow_shoulder', Pose, self.get_elbow_shoulder)
    left_neck_shoulder_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/left_neck_shoulder', Pose, self.get_neck_shoulder)
    left_torso_shou_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/left_torso_shoulder', Pose, self.get_torso_shoulder)
    left_right_shou_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/left_right_shoulder', Pose, self.get_left_right_shoulder)
    # Used to get the state of the calibration mode from a ROS Topic
    mode_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/robot_mode', Int32, self.get_mode)

    # Subscribe to wrist sensor
    wrist_sub = rospy.Subscriber('wrist_euler', Float64MultiArray, self.get_wrist)

    # Subscribe to calibration angle
    calib_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/angle_calibrate', Float64MultiArray, self.get_calib)

    # Subscribe to upper arm sagital and coronal sign
    sagstat_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/upper_arm_sagstat', Int32, self.get_sagstat)
    corstat_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/upper_arm_corstat', Int32, self.get_corstat)

    # Topics publisher initialization
    # Publisher of human's arm length
    left_arm_pub = rospy.Publisher('/fourbythree_topics/ergonomics/left_arm', Float64MultiArray, queue_size = 10)
    # Publisher of human's joint angle
    left_angle_pub = rospy.Publisher('/fourbythree_topics/ergonomics/left_human_angle', Float64MultiArray, queue_size = 10)

    # Setting the data rate as 40 Hz
    r = rospy.Rate(40.0)
    while not rospy.is_shutdown():
      #   Calculate the human's joint angle from human skeleton
      self.calculate_angle()

      #   Publish the human's joint angle
      left_angle_pub.publish(self.left_angle)
      #   Publish the human's arm length
      left_arm_pub.publish(self.left_arm)
      # Sleep until 1/40 s
      r.sleep()

  def get_mode(self,vec):
      # Check whether the human is in calibration mode
      self.mode = vec.data

  def get_hand_elbow(self,vec):
    # This is used to receive the vector position of hand with respect to shoulder
    self.left_hand_shoulder.position.x = vec.position.x
    self.left_hand_shoulder.position.y = vec.position.y
    self.left_hand_shoulder.position.z = vec.position.z

  def get_elbow_shoulder(self,vec):
    # This is used to receive the vector position of elbow  with respect to shoulder
    self.left_elbow_shoulder.position.x = vec.position.x
    self.left_elbow_shoulder.position.y = vec.position.y
    self.left_elbow_shoulder.position.z = vec.position.z

  def get_neck_shoulder(self,vec):
    # This is used to receive the vector position of neck with respect to shoulder
    self.left_neck_shoulder.position.x = vec.position.x
    self.left_neck_shoulder.position.y = vec.position.y
    self.left_neck_shoulder.position.z = vec.position.z

  def get_torso_shoulder(self,vec):
    # This is used to receive the vector position of torso with respect to shoulder
    self.left_torso_shoulder.position.x = vec.position.x
    self.left_torso_shoulder.position.y = vec.position.y
    self.left_torso_shoulder.position.z = vec.position.z

  def get_left_right_shoulder(self,vec):
    # This is used to receive the vector position of right shoulder with respect to left shoulder
    self.left_right_shoulder.position.x = vec.position.x
    self.left_right_shoulder.position.y = vec.position.y
    self.left_right_shoulder.position.z = vec.position.z

  def get_wrist(self,vec):
    # This function is used to receive the wrist angle value

    # Executed when the wrist sensor is used
    if(self.wrist_on==1):
        # Executed when the human is in calibrated state
        if(self.mode == 0):
            # Read wrist angle in radian and convert to degree. For wrist_angle[0], the value is inverted to make it consistent with reality
            self.wrist_angle[0] = -1*vec.data[0]*180.0/math.pi
            self.wrist_angle[1] = vec.data[2]*180.0/math.pi

        # Executed when the human is not in calibrated state, i.e. calibration is finished
        else:
            # Read wrist angle in radian and convert to degree then substracted by the offset value from the calibration mode
            self.wrist_angle[0] = -1*vec.data[0]*180.0/math.pi+self.wrist_init[0]
            self.wrist_angle[1] = (vec.data[2]*180.0/math.pi-self.wrist_init[1])

  def get_calib(self,vec):
    # This function is used to get the offset value from calibration mode

    # The human's joint angle is as follows
    # 1. Theta: lower arm Sagital Angle
    # 2. Alpha: upper arm Sagital Angle
    # 3. Gamma: upper arm Coronal angle
    # 3. Phi: lower arm Coronal angle
    # 4. wrist_init[0]-[1]: Wrist angles. Don't forget to invert the value of wrist_init[0] to make it consistent with reality
    self.theta_init = vec.data[0]
    self.alpha_init = vec.data[1]
    self.gamma_init = vec.data[4]
    self.wrist_init[0] = -1*vec.data[2]
    self.wrist_init[1] = vec.data[3]
    self.phi_init = vec.data[5]

  def get_sagstat(self,vec):
    # Get the sign of the sagital angle
    self.sagital_status = vec.data

  def get_corstat(self,vec):
    # Get the sign of the coronal angle
    self.coronal_status = vec.data

  def calculate_angle(self):
    # Get the human's arm vector in the 3x1 matrix form
    left_hand_elbow_vector = np.matrix([[self.left_hand_shoulder.position.x-self.left_elbow_shoulder.position.x], [self.left_hand_shoulder.position.y-self.left_elbow_shoulder.position.y], [self.left_hand_shoulder.position.z-self.left_elbow_shoulder.position.z]])
    left_elbow_shoulder_vector = np.matrix([[self.left_elbow_shoulder.position.x], [self.left_elbow_shoulder.position.y], [self.left_elbow_shoulder.position.z]])
    left_torso_neck_vector = np.matrix([[self.left_torso_shoulder.position.x-self.left_neck_shoulder.position.x], [self.left_torso_shoulder.position.y-self.left_neck_shoulder.position.y], [self.left_torso_shoulder.position.z-self.left_neck_shoulder.position.z]])
    left_neck_shoulder_vector = np.matrix([[self.left_right_shoulder.position.x], [self.left_right_shoulder.position.y], [self.left_right_shoulder.position.z]])

    # The human's joint angle is as follows
    # 1. Theta: lower arm Sagital Angle
    # 2. Alpha: upper arm Sagital Angle
    # 3. Gamma: upper arm Coronal angle
    # 4. Phi: lower arm Coronal angle

    # Make sure that the vector's magnitude is not 0
    if((linalg.norm(left_hand_elbow_vector)*linalg.norm(left_elbow_shoulder_vector))!=0):
      # Get the cosine of angle theta from hand-elbow vector and elbow-shoulder vector
      cos_theta = (left_hand_elbow_vector.getT()*left_elbow_shoulder_vector)/(linalg.norm(left_hand_elbow_vector)*linalg.norm(left_elbow_shoulder_vector))
      # Get the angle theta and convert to degree
      left_theta = math.acos(cos_theta)*180.0/math.pi
    else:
      # If the magnitude of the corresponding vector is 0, set theta as 0
      cos_theta = 1.0
      left_theta = 0.0

    # Make sure that the vector's magnitude is not 0
    if((linalg.norm(left_torso_neck_vector)*linalg.norm(left_elbow_shoulder_vector))!=0):
      # Get the cosine of angle alpha from torso-neck vector and elbow-shoulder vector
      cos_alpha = (left_torso_neck_vector.getT()*left_elbow_shoulder_vector)/(linalg.norm(left_torso_neck_vector)*linalg.norm(left_elbow_shoulder_vector))
      # Get the angle alpha and convert to degree
      left_alpha = math.acos(cos_alpha)*180.0/math.pi
    else:
      # If the magnitude of the corresponding vector is 0, set alpha as 0
      cos_alpha = 1.0
      left_alpha = 0.0

    # Make sure that the vector's magnitude is not 0
    if((linalg.norm(left_elbow_shoulder_vector)*linalg.norm(left_neck_shoulder_vector))!=0):
      # Get the cosine of angle gamma from neck-shoulder vector and elbow-shoulder vector
      cos_gamma = (left_neck_shoulder_vector.getT()*left_elbow_shoulder_vector)/(linalg.norm(left_neck_shoulder_vector)*linalg.norm(left_elbow_shoulder_vector))
      # Get the angle gamma and convert to degree and subtracted by 90 degree
      left_gamma = math.acos(cos_gamma)*180.0/math.pi - 90.0
    else:
      # If the magnitude of the corresponding vector is 0, set gamma as -90 degree
      cos_gamma = 1.0
      left_gamma = -90.0

    # Make sure that the vector's magnitude is not 0
    if((linalg.norm(left_hand_elbow_vector)*linalg.norm(left_neck_shoulder_vector))!=0):
      # Get the cosine of angle phi from hand-elbow vector and neck-shoulder vector
      cos_phi = (-1*left_neck_shoulder_vector.getT()*left_hand_elbow_vector)/(linalg.norm(left_neck_shoulder_vector)*linalg.norm(left_hand_elbow_vector))
      # Get the angle phi and convert to degree
      left_phi =  90.0 - math.acos(cos_phi)*180.0/math.pi
    else:
      # If the magnitude of the corresponding vector is 0, set phi as 90 degree
      cos_phi = 1.0
      left_phi = 90.0

    # Get the upper and lower arm's length
    self.a = linalg.norm(left_elbow_shoulder_vector)
    self.b = linalg.norm(left_hand_elbow_vector)
    # Put the arm's length into array
    self.left_arm.data = []
    self.left_arm.data.append(self.a)
    self.left_arm.data.append(self.b)

    # Empty the array for human's joint angle
    self.left_angle.data = []

    if(self.mode==0):
        # Executed while in calibration mode

        # Put all the human's joint angles into array
        self.left_angle.data.append(left_theta)
        self.left_angle.data.append(left_alpha)
        self.left_angle.data.append(self.wrist_angle[0])
        self.left_angle.data.append(self.wrist_angle[1])
        self.left_angle.data.append(left_gamma)
        self.left_angle.data.append(left_phi)
    else:
        # Executed while not in calibration mode

        # Normalize the angle theta by adding 90 degree minus the offset from calibration
        self.left_angle.data.append(90.0+left_theta-self.theta_init)
        # Check the sign of coronal angle
        if(self.coronal_status == 0):
            # Put the angle alpha minus the offset from calibration multiply by the sagital sign into the array
            self.left_angle.data.append((left_alpha-self.alpha_init)*self.sagital_status)
        else:
            # Put the angle alpha minus the offset from calibration multiply by the sagital sign into the array
            self.left_angle.data.append((left_alpha-self.alpha_init)*self.coronal_status)
        # Put the wrist angle into the array. Calibration offset already subtracted from the angle.
        self.left_angle.data.append(self.wrist_angle[0])
        self.left_angle.data.append(self.wrist_angle[1])
        # Put the angle gamma and phi  minus the offset from calibration into the array
        self.left_angle.data.append(left_gamma-self.gamma_init)
        self.left_angle.data.append(left_phi-self.phi_init)

if __name__ == '__main__':
    try:
        human_angle_node()
    except rospy.ROSInterruptException: pass
