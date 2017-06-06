#!/usr/bin/env python
# The code is used to generate a sign for the human's joint angle.
import roslib
roslib.load_manifest('kcl_ergonomics')
import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray

class human_position(object):
  def __init__(self):
    rospy.init_node('human_position')

    # This variable gets the human index detected by the Kinect from parameter 'ind'
    index = rospy.get_param('~ind')
    # Initialize the TransformListener TF class
    listener = tf.TransformListener()

    # Initialize several flags
    upper_arm_flag = Int32()
    upper_arm_sagflag = Int32()
    lower_arm_flag = Int32()

    # Initialize several variables
    hip_elbow = Float64MultiArray()
    cor_calib = 0.0
    sag_calib = 0.0
    lower_calib = 0.0
    cor_save = 0.0
    sag_save = 0.0
    lower_save = 0.0

    mode_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/robot_mode', Int32, self.get_mode)

    # There are upper arm coronal, upper arm sagital, lower arm
    # Initilize the topics producing the sign for the human angles.
    corstat_pub = rospy.Publisher('/fourbythree_topics/ergonomics/upper_arm_corstat', Int32, queue_size = 10)
    sagstat_pub = rospy.Publisher('/fourbythree_topics/ergonomics/upper_arm_sagstat', Int32, queue_size = 10)
    lower_pub = rospy.Publisher('/fourbythree_topics/ergonomics/lower_arm_stat', Int32, queue_size = 10)

    # Initilize the topics producing the offset distance between hip and elbow
    hipelbow_pub = rospy.Publisher('/fourbythree_topics/ergonomics/hip_elbow', Float64MultiArray, queue_size = 10)

    # Set the data rate to be 40 Hz
    rate = rospy.Rate(40.0)

    # Initilize 3 flags used to check whether the listeners have started
    flag_1 = 0
    flag_2 = 0
    flag_3 = 0

    # Initilize the high and low limit for changing the angle sign
    high_limit = 0.0
    low_limit = -0.2
    high_limit3 = 0.15
    low_limit3 = -0.15
    # Read whether the human is in calibrated state
    self.mode = rospy.get_param('/robot_mode')

    while not rospy.is_shutdown():
        # Listen to the frame left_elbow
        try:
            now = rospy.Time.now()
            (trans,rot) = listener.lookupTransform('openni_link', 'left_elbow_'+str(index), rospy.Time(0))
            flag_1 = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Listen to the frame left_hip
        try:
            now = rospy.Time.now()
            (trans2,rot2) = listener.lookupTransform('openni_link', 'left_hip_'+str(index), rospy.Time(0))
            flag_2 = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Listen to the frame left_hand
        try:
            now = rospy.Time.now()
            (trans3,rot3) = listener.lookupTransform('openni_link', 'left_hand_'+str(index), rospy.Time(0))
            flag_3 = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print flag_1
        print flag_2
        print flag_3
        # Checking whether all frames have been received
        if(flag_1 == 1 and flag_2 == 1 and flag_3==1):
            # Checking the position difference in x-axis between hip and elbow to decide the sign of upper arm angle
            diff = trans[0] - trans2[0] - cor_calib

            # When the difference is higher than high limit, assign -1 as a sign
            if(diff>high_limit):
                upper_arm_flag.data = -1
            # When the difference is between low and high limit, assign 0 as a sign
            elif(diff<=high_limit and diff > low_limit):
                upper_arm_flag.data = 0
            # When the difference is less than lower limit, assign 1 as a sign
            else:
                upper_arm_flag.data = 1


            # Checking the position difference in y-axis between hip and elbow to decide the sign of upper arm sagital angle
            diff2 = trans[1] - trans2[1] - sag_calib

            # When the difference is positive, assign -1 as a sign
            if(diff2>0):
                upper_arm_sagflag.data = -1
            # When the difference is negative, assign 1 as a sign
            else:
                upper_arm_sagflag.data = 1


            # Checking the position difference in x-axis between hand and elbow to decide the sign of lower arm angle
            diff3 = trans3[0] - trans[0] - lower_calib

            # When the difference is higher than high limit, assign 1 as a sign
            if(diff3>high_limit3):
                lower_arm_flag.data = 1
            # When the difference is between low and high limit, assign 0 as a sign
            elif(diff3<=high_limit3 and diff3 > low_limit3):
                lower_arm_flag.data = 0
            # When the difference is less than lower limit, assign 1 as a sign
            else:
                lower_arm_flag.data = 1

            # Check whether the human is in calibrated state
            if(self.mode==0):
                # Read the current offset value
                cor_save = diff
                sag_save = diff2
                lower_save = diff3
                cor_calib = 0.0
                sag_calib = 0.0
                lower_calib = 0.0
            else:
                # Save the offset value derived from calibration
                cor_calib = cor_save
                sag_calib = sag_save
                lower_calib = lower_save

            # Save the distance between hip and elbow to be sent as topic
            hip_elbow.data = []
            hip_elbow.data.append(diff)
            hip_elbow.data.append(diff2)

        # Publish the angle sign
        corstat_pub.publish(upper_arm_flag)
        sagstat_pub.publish(upper_arm_sagflag)
        lower_pub.publish(lower_arm_flag)

        # Publish the hip-elbow distance
        hipelbow_pub.publish(hip_elbow)

        # Sleep until 1/40 s
        rate.sleep()

  def get_mode(self,vec):
      # Check whether the human is in calibration mode
      self.mode = vec.data

if __name__ == '__main__':
    try:
        human_position()
    except rospy.ROSInterruptException: pass
