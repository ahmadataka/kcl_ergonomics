#!/usr/bin/env python
# This code is used to generate the new target position and orientation for the Robot based on the human ergonomic score


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

# This function is used to set the order of priority for the robot's movement
def order_move(index_count):
    if(index_count==0):
        # The first priority is the Upper Arm Coronal
        index_out = 6
    elif(index_count==1):
        # The second priority is the Lower Arm Coronal
        index_out = 1
    elif(index_count==2):
        # The third priority is the Wrist
        index_out = 4
    elif(index_count==3):
        # The fourth priority is the Upper Arm Sagital
        index_out = 3
    elif(index_count==4):
        # The fifth priority is the Lower Arm Sagital
        index_out = 2

    # Return the index used to decide which movement the robot should do
    return index_out

# This is the main function
class ergonomic_pose(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('ergonomic_pose')
    # Read the calibration mode
    self.calib_mode = rospy.get_param('/robot_mode')
    # Initialize the human's joint angle
    self.left_angle = [70.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Define the pose class used to send the target to the robot
    self.pose_sent = Pose()
    # Initialize the state of the robot's movement
    self.state = 0
    # Initialize the timing of the robot to wait before deciding to move or not to move
    self.delta = 0.0
    # Set the waiting time for the robot
    self.time_bound = 1.0
    # Initialize the vector of the robot's movement
    self.dist = [0.0, 0.0, 0.0]

    # Initialize the upper arm's length
    self.upper_length = 0.075
    self.upper_length_2 = 0.125
    # Initialize the lower arm's length
    self.lower_length = 0.10
    self.lower_length_2 = 0.2

    # Initialize the target's position for the robot
    self.target = [0.0, 0.0, 0.0]

    # Initialize the robot's Euler angle
    self.euler = [0.0, 0.0, 0.0]
    # Initialize the target's Euler angle
    self.eul_target = [0.0, 0.0, 0.0]
    # Initialize the flag used to show status whether the new target has been assigned or not
    self.assign_target = 0

    # Set the boundary of Rula Score
    self.rula_bound = 2
    # Initialize the Rula Score, Upper Arm scoere, Lower Arm Score, and Wrist Score
    self.rula = 1
    self.upper = 1
    self.lower = 1
    self.wrist = 1

    # Initialize the variable used to determine the order of priority for robot movement
    self.counter = 0

    # Initialize the individual score
    self.rula_ind = 1

    # Initialize the abduction and lateral sign
    self.abduct_stat = 0
    self.lateral_stat = 0

    # Initialize the flag showing the status of whether the task has been finished
    self.task_stat = 0
    # Initialize variable used as a flag to start the timer for robot's waiting time
    self.state_count = 0

    # Define the ROS Subscriber
    # Used to receive the human's joint angle
    left_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/left_human_angle', Float64MultiArray, self.get_left)
    # Used to receive the robot's Euler angle
    euler_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/euler_gripper', Float64MultiArray, self.get_euler)
    # Used to receive the Rula score
    rula_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/rula_score', Float64MultiArray, self.get_rula)
    # Used to receive the upper arm score
    upper_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/upper_arm_score', Float64MultiArray, self.get_upper)
    # Used to receive the lower arm score
    lower_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/lower_arm_score', Float64MultiArray, self.get_lower)
    # Used to receive the wrist score
    wrist_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/wrist_score', Float64MultiArray, self.get_wrist)
    # Used to receive the wrist twist score
    wrist_twist_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/wrist_twist_score', Float64MultiArray, self.get_wrist_tw)
    # Used to receive the coronal upper arm status / sign
    abduct_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/upper_arm_corstat', Int32, self.get_abduct)
    # Used to receive the lower arm status / sign
    lateral_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/lower_arm_stat', Int32, self.get_lateral)
    # Used to receive the flag to check whether the robot has finished the task
    task_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/task_finish_flag', Int32, self.get_task)
    # Used to receive the arm's length after calibration
    calib_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/arm_calibrate', Float64MultiArray, self.get_arm)

    # Define the ROS Publisher
    # Used to publish the robot's target pose
    self.pose_pub = rospy.Publisher('/fourbythree_topics/ergonomics/baxter_target', Pose, queue_size = 10)

    # Set the rate to be 40 Hz
    r = rospy.Rate(40.0)
    while not rospy.is_shutdown():
      # Check whether the human is in calibration mode
      self.calib_mode = rospy.get_param('/robot_mode')
      # Execute the ergonomics algorithm when the human is not in calibration mode
      if(self.calib_mode == 1):
          self.full_ergo()
      # Otherwise, reset all the variables
      else:
          self.assign_target = 0
          self.state = 0
          self.delta = 0.0
          self.counter = 0

      # Sleep until 1/40 s
      r.sleep()

  # Used to receive the human's joint angle
  def get_left(self,vec):
    self.left_angle[0] = vec.data[0]
    self.left_angle[1] = vec.data[1]
    self.left_angle[2] = vec.data[2]
    self.left_angle[3] = vec.data[3]
    self.left_angle[4] = vec.data[4]
    self.left_angle[5] = vec.data[5]

  # Used to receive the arm's length
  def get_arm(self,vec):
    self.upper_length = self.upper_length_2 = vec.data[0]
    self.lower_length = self.lower_length_2 = vec.data[1]

  # Used to receive the robot's Euler angle
  def get_euler(self,vec):
    self.euler[0] = vec.data[0]
    self.euler[1] = vec.data[1]
    self.euler[2] = vec.data[2]

  # Used to receive the Rula score
  def get_rula(self,vec):
    self.rula = vec.data[0]
  # Used to receive the upper arm score
  def get_upper(self,vec):
    self.upper = vec.data[0]
  # Used to receive the lower arm score
  def get_lower(self,vec):
    self.lower = vec.data[0]
  # Used to receive the wrist score
  def get_wrist(self,vec):
    self.wrist = vec.data[0]
  # Used to receive the wrist twist score
  def get_wrist_tw(self,vec):
    self.wrist_twist = vec.data[0]
  # Used to receive the lateral sign
  def get_lateral(self,vec):
    self.lateral_stat = vec.data
  # Used to receive the abduction sign
  def get_abduct(self,vec):
    self.abduct_stat = vec.data
  # Used to receive the flag showing whether the task has been completed
  def get_task(self,vec):
    self.task_stat = vec.data

  # Used to calculate the amount of movement needed for the robot to move according to the Rula score
  def calculate_pose(self):
    self.pose_sent.position.x = self.target[0]
    self.pose_sent.position.y = self.target[1]
    self.pose_sent.position.z = self.target[2]

    # The state for the lower arm coronal correction
    if(self.state == 1):
        # Reset the state and waiting time
        self.state = 0
        self.delta = 0.0

        # Calculate the movement in x,y,z direction
        self.dist[0] = -self.lower_length_2*(1.0-math.cos((self.left_angle[5])*math.pi/180.0))
        self.dist[1] = -self.lower_length_2*math.sin((self.left_angle[5])*math.pi/180.0)
        self.dist[2] = 0.0

        # Save the calculated movement
        self.target[0] = self.dist[0]
        self.target[1] = self.dist[1]
        self.target[2] = self.dist[2]

        # Save the calculated movement into a Pose type message for publisher
        self.pose_sent.position.x = self.target[0]
        self.pose_sent.position.y = self.target[1]
        self.pose_sent.position.z = self.target[2]
        self.pose_sent.orientation.x = 0.0
        self.pose_sent.orientation.y = 0.0
        self.pose_sent.orientation.z = 0.0
        self.pose_sent.orientation.w = 0.0

        # Set the assign_target flag, showing that the new target has been assigned
        self.assign_target = 1
        print "assign lower coronal"
        # Publish the pose correction
        self.pose_pub.publish(self.pose_sent)

    # The state for the lower arm sagital correction
    elif(self.state == 2):
        # Reset the state and waiting time
        self.state = 0
        self.delta = 0.0

        # Calculate the movement in x,y,z direction
        self.dist[0] = -self.lower_length*(1.0-math.sin((self.left_angle[0])*math.pi/180.0))
        self.dist[1] = 0.0
        self.dist[2] = self.lower_length*math.cos((self.left_angle[0])*math.pi/180.0)

        # Save the calculated movement
        self.target[0] = self.dist[0]
        self.target[1] = self.dist[1]
        self.target[2] = self.dist[2]

        # Save the calculated movement into a Pose type message for publisher
        self.pose_sent.position.x = self.target[0]
        self.pose_sent.position.y = self.target[1]
        self.pose_sent.position.z = self.target[2]
        self.pose_sent.orientation.x = 0.0
        self.pose_sent.orientation.y = 0.0
        self.pose_sent.orientation.z = 0.0
        self.pose_sent.orientation.w = 0.0

        # Set the assign_target flag, showing that the new target has been assigned
        self.assign_target = 1
        print "assign lower sagital"
        # Publish the pose correction
        self.pose_pub.publish(self.pose_sent)

    # The state for the upper arm sagital correction
    elif(self.state == 3):
        # Reset the state and waiting time
        self.state = 0
        self.delta = 0.0

        # Calculate the movement in x,y,z direction
        self.dist[0] = self.upper_length_2*math.sin(self.left_angle[1]*math.pi/180.0)
        self.dist[1] = 0.0
        self.dist[2] = self.upper_length_2*(math.cos(self.left_angle[1]*math.pi/180.0)-1.0)

        # Save the calculated movement
        self.target[0] = self.dist[0]
        self.target[1] = self.dist[1]
        self.target[2] = self.dist[2]

        # Save the calculated movement into a Pose type message for publisher
        self.pose_sent.position.x = self.target[0]
        self.pose_sent.position.y = self.target[1]
        self.pose_sent.position.z = self.target[2]
        self.pose_sent.orientation.x = 0.0
        self.pose_sent.orientation.y = 0.0
        self.pose_sent.orientation.z = 0.0
        self.pose_sent.orientation.w = 0.0

        # Set the assign_target flag, showing that the new target has been assigned
        self.assign_target = 1
        print "assign upper sagital"
        # Publish the pose correction
        self.pose_pub.publish(self.pose_sent)

    # The state for the wrist correction
    elif(self.state == 4):
        # Reset the state and waiting time
        self.state = 0
        self.delta = 0.0

        # Assign the new orientation target
        self.eul_target[0] = self.euler[0]
        self.eul_target[1] = self.euler[1]
        self.eul_target[2] = self.euler[2]+(self.left_angle[2]*3.14/180.0)

        print "assign wrist"
        # Set the assign_target flag, showing that the new target has been assigned
        self.assign_target = 1

        # Transform from Euler angle to quaternion
        quat_target = tf.transformations.quaternion_from_euler(self.eul_target[0],self.eul_target[1],self.eul_target[2])
        # Save the calculated movement into a Pose type message for publisher
        self.pose_sent.orientation.x = quat_target[0]
        self.pose_sent.orientation.y = quat_target[1]
        self.pose_sent.orientation.z = quat_target[2]
        self.pose_sent.orientation.w = quat_target[3]
        self.pose_sent.position.x = 0.0
        self.pose_sent.position.y = 0.0
        self.pose_sent.position.z = 0.0
        # Publish the pose correction
        self.pose_pub.publish(self.pose_sent)

    # The state for the wrist twist correction
    elif(self.state == 5):
        # Reset the state and waiting time
        self.state = 0
        self.delta = 0.0

        # Calculate the movement in x,y,z direction
        self.dist[0] = -abs(self.upper_length_2*math.tan(self.left_angle[3]*math.pi/180.0/2)*math.sin(self.left_angle[3]*math.pi/180.0))
        self.dist[1] = 0.0
        if(self.left_angle[3]<0):
            self.dist[2] = -1*self.upper_length_2*math.sin(self.left_angle[3]*math.pi/180.0)
        elif(self.left_angle[3]>0):
            self.dist[2] = -self.upper_length_2*math.sin(self.left_angle[3]*math.pi/180.0)

        # Save the calculated movement
        self.target[0] = self.dist[0]
        self.target[1] = self.dist[1]
        self.target[2] = self.dist[2]

        # Save the calculated movement into a Pose type message for publisher
        self.pose_sent.position.x = self.target[0]
        self.pose_sent.position.y = self.target[1]
        self.pose_sent.position.z = self.target[2]
        self.pose_sent.orientation.x = 0.0
        self.pose_sent.orientation.y = 0.0
        self.pose_sent.orientation.z = 0.0
        self.pose_sent.orientation.w = 0.0

        # Set the assign_target flag, showing that the new target has been assigned
        self.assign_target = 1
        print "assign wrist up"
        # Publish the pose correction
        self.pose_pub.publish(self.pose_sent)

    # The state for the upper arm coronal correction
    elif(self.state == 6):
        # Reset the state and waiting time
        self.state = 0
        self.delta = 0.0

        # Calculate the movement in x,y,z direction
        self.dist[0] = 0.0
        self.dist[1] = -self.upper_length_2*math.sin((self.left_angle[1])*math.pi/180.0)
        self.dist[2] = self.upper_length_2*(math.cos(self.left_angle[1]*math.pi/180.0)-1.0)

        # Save the calculated movement
        self.target[0] = self.dist[0]
        self.target[1] = self.dist[1]
        self.target[2] = self.dist[2]

        # Save the calculated movement into a Pose type message for publisher
        self.pose_sent.position.x = self.target[0]
        self.pose_sent.position.y = self.target[1]
        self.pose_sent.position.z = self.target[2]
        self.pose_sent.orientation.x = 0.0
        self.pose_sent.orientation.y = 0.0
        self.pose_sent.orientation.z = 0.0
        self.pose_sent.orientation.w = 0.0

        # Set the assign_target flag, showing that the new target has been assigned
        self.assign_target = 1
        print "assign upper coronal"
        # Publish the pose correction
        self.pose_pub.publish(self.pose_sent)

  # Used to run the ergonomics algorithm based on priority
  def full_ergo(self):
      # Reset the flag used to move to the next correction priority
      self.increase = 0
      # Check whether the Rula score is bigger than the boundary and whether the previous movement has been finished
      if(self.rula>=self.rula_bound and self.task_stat == 0):
          # Check whether the previous movement has been finished
          if(self.task_stat==0):
              # Check whether the counting has been started
              if(self.state_count == 0):
                # Start the counter and set the flag to show that the counting has been started
                self.begin = rospy.Time.now()
                self.state_count = 1

              # Read the current time
              self.current = rospy.Time.now()
              # Calculate the waiting time of the robot before starting to determina the movement correction if the RULA continues to be big
              self.delta = float(self.current.secs)+float(self.current.nsecs)*(10.0**(-9.0)) - (float(self.begin.secs)+float(self.begin.nsecs)*(10.0**(-9.0)))

          # Run when the waiting time is bigger than the boundary
          if(self.delta>self.time_bound):
              # Reset the state number before starting
              self.state = 0
              # Check whether the target has been assigned
              while(self.assign_target == 0):
                  # If the target hasn't been assigned yet, run the ergonomics-based movement correction

                  # Reset the flag used to move to the next correction priority
                  self.increase = 0
                  # Produce the mode number according to the priority
                  self.mode = order_move(self.counter)

                  # Mode = 4 and Mode =5 are for wrist correction
                  if(self.mode==4 or self.mode==5):
                      # Get the wrist score
                      self.rula_ind = self.wrist
                  # Mode = 3 and Mode =6 are for upper arm correction
                  elif(self.mode==3 or self.mode==6):
                      # Get the upper arm score
                      self.rula_ind = self.upper
                  # Mode = 2 and Mode =1 are for lower arm correction
                  elif(self.mode==2 or self.mode==1):
                      # Get the lower arm score
                      self.rula_ind = self.lower

                  # Check whether the current individual score is bigger than the boundary
                  if(self.rula_ind > 1.0):

                    # Check which mode is running according to the priority
                    if(self.mode == 1):
                      # When mode=1, check whether the high individual score is due to the lateral
                      if(self.lateral_stat==1 ):
                        # If so, choose coronal lower arm
                  	    self.state = 1
                      else:
                            # If not, set the flag 'increase' so that the algorithm check the next angle in the priority order
                            self.increase = 1
                            self.target[0] = 0.0
                            self.target[1] = 0.0
                            self.target[2] = 0.0
                    # When mode=2, check whether the high individual score is due to the lateral
                    elif(self.mode == 2):
                      if(self.lateral_stat==0):
                        # If not, choose sagital lower arm
                        self.state = 2
                      else:
                            # If so, set the flag 'increase' so that the algorithm check the next angle in the priority order
                            self.increase = 1
                            self.target[0] = 0.0
                            self.target[1] = 0.0
                            self.target[2] = 0.0

                    # When mode=3, check whether the high individual score is due to the abduction
                    elif(self.mode == 3):
                      if(self.abduct_stat==0):
                        # If not, choose sagital upper arm
                  	    self.state = 3
                      else:
                            # If so, set the flag 'increase' so that the algorithm check the next angle in the priority order
                            self.increase = 1
                            self.target[0] = 0.0
                            self.target[1] = 0.0
                            self.target[2] = 0.0

                    elif(self.mode == 4):
                      self.state = 4

                    elif(self.mode == 5):
                      self.state = 5

                    # When mode=6, check whether the high individual score is due to the abduction
                    elif(self.mode == 6):
                      if(self.abduct_stat!=0):
                        # If so, choose coronal upper arm
                  	    self.state = 6
                      else:
                        # If not, set the flag 'increase' so that the algorithm check the next angle in the priority order
                        self.increase = 1
                        self.target[0] = 0.0
                        self.target[1] = 0.0
                        self.target[2] = 0.0

                    # Calculate the amount of correction according to the active mode
                    self.calculate_pose()

                  else:
                      # When the individual score is not big enough, set the flag 'increase' so that the algorithm check the next angle in the priority order
                      self.increase = 1

                  # When flag 'increase' is set, increase variable counter so that the next angle can be checked or if all angles have been checked, reset the counter
                  if(self.increase == 1):
                    if(self.counter!=4):
                        self.counter = self.counter+1
                    else:
                        self.counter = 0

              # Reset the state_count after new target has been assigned, meaning that the next counter of robot waiting time can be started once the Rula score is detected to be high
              self.state_count = 0

          # Check whether the new target has been assigned
          if(self.assign_target == 1):
              # If so, reset the flag and reset the counter used to check all the angles so that the checking can be done from the beginning of the priority order
              self.assign_target = 0
              self.counter = 0

      else:
          # When the overall Rula is not high enough, just reset all the flag variables
          self.assign_target = 0
          self.state = 0
          self.delta = 0.0
          self.counter = 0
          self.state_count = 0



if __name__ == '__main__':
    try:
        ergonomic_pose()
    except rospy.ROSInterruptException: pass
