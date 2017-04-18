#!/usr/bin/env python
import roslib
roslib.load_manifest('kcl_ergonomics')
import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':
    rospy.init_node('human_position')

    index = rospy.get_param('~ind')
    listener = tf.TransformListener()
    upper_arm_flag = Int32()
    upper_arm_sagflag = Int32()
    lower_arm_flag = Int32()
    hip_elbow = Float64MultiArray()
    cor_calib = 0.0
    sag_calib = 0.0
    lower_calib = 0.0
    cor_save = 0.0
    sag_save = 0.0
    lower_save = 0.0
    corstat_pub = rospy.Publisher('upper_arm_corstat', Int32, queue_size = 10)
    sagstat_pub = rospy.Publisher('upper_arm_sagstat', Int32, queue_size = 10)
    lower_pub = rospy.Publisher('lower_arm_stat', Int32, queue_size = 10)
    hipelbow_pub = rospy.Publisher('hip_elbow', Float64MultiArray, queue_size = 10)
    rate = rospy.Rate(40.0)
    flag_1 = 0
    flag_2 = 0
    flag_3 = 0
    high_limit = 0.0
    low_limit = -0.2
    high_limit3 = 0.15
    low_limit3 = -0.15
    while not rospy.is_shutdown():
	#(trans,rot) = listener.lookupTransform('openni_depth_frame', 'openni_depth_optical_frame', rospy.Time(0))
        mode = rospy.get_param('/robot_mode')
        try:
            now = rospy.Time.now()
            (trans,rot) = listener.lookupTransform('openni_link', 'left_elbow_'+str(index), rospy.Time(0))
            flag_1 = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            now = rospy.Time.now()
            (trans2,rot2) = listener.lookupTransform('openni_link', 'left_hip_'+str(index), rospy.Time(0))
            flag_2 = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            now = rospy.Time.now()
            (trans3,rot3) = listener.lookupTransform('openni_link', 'left_hand_'+str(index), rospy.Time(0))
            flag_3 = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        if(flag_1 == 1 and flag_2 == 1 and flag_3==1):
            diff = trans[0] - trans2[0] - cor_calib
            if(diff>high_limit):
                upper_arm_flag.data = -1
            elif(diff<=high_limit and diff > low_limit):
                upper_arm_flag.data = 0
            else:
                upper_arm_flag.data = 1
            diff2 = trans[1] - trans2[1] - sag_calib
            if(diff2>0):
                upper_arm_sagflag.data = -1
            else:
                upper_arm_sagflag.data = 1
            diff3 = trans3[0] - trans[0] - lower_calib
            if(diff3>high_limit3):
                lower_arm_flag.data = 1
            elif(diff3<=high_limit3 and diff3 > low_limit3):
                lower_arm_flag.data = 0
            else:
                lower_arm_flag.data = 1
            if(mode==0):
                cor_save = diff
                sag_save = diff2
                lower_save = diff3
                cor_calib = 0.0
                sag_calib = 0.0
                lower_calib = 0.0
            else:
                cor_calib = cor_save
                sag_calib = sag_save
                lower_calib = lower_save
            # print diff3
            hip_elbow.data = []
            hip_elbow.data.append(diff)
            hip_elbow.data.append(diff2)

        corstat_pub.publish(upper_arm_flag)
        sagstat_pub.publish(upper_arm_sagflag)
        lower_pub.publish(lower_arm_flag)
        hipelbow_pub.publish(hip_elbow)
        rate.sleep()
