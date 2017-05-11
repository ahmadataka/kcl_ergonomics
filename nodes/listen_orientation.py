#!/usr/bin/env python
# The code is used to produce the Euler angle of the robot's gripper.
import roslib
roslib.load_manifest('kcl_ergonomics')
import rospy
import tf
from std_msgs.msg import Float64MultiArray

class listen_orientation(object):
    def __init__(self):
        rospy.init_node('listen_orientation')

        active_frame = '/left_gripper'
        reference_frame = '/torso'
        topic_name = '/fourbythree_topics/ergonomics/euler_gripper'
        listener = tf.TransformListener()

        tip_pose = rospy.Publisher(topic_name, Float64MultiArray, queue_size = 10)
        rate = rospy.Rate(40.0)
        while not rospy.is_shutdown():
    	#(trans,rot) = listener.lookupTransform('openni_depth_frame', 'openni_depth_optical_frame', rospy.Time(0))

            try:
    	        now = rospy.Time.now()
                (trans,rot) = listener.lookupTransform(reference_frame, active_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            euler = tf.transformations.euler_from_quaternion(rot)
    	    pose = Float64MultiArray()
            for i in range(0,3):
                pose.data.append(euler[i])

            tip_pose.publish(pose)

            rate.sleep()

if __name__ == '__main__':
    try:
        listen_orientation()
    except rospy.ROSInterruptException: pass
