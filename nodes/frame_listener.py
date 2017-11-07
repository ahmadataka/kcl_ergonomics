#!/usr/bin/env python
# The code is used to publish to a topic consisting the pose of a target frame with respect to a reference frame
# Created by King's College London and Queen Mary University of London, 2017.
import roslib
roslib.load_manifest('kcl_ergonomics')
import rospy
import tf
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    rospy.init_node('frame_listener')

    # This variable gets the target frame from parameter 'frame'
    active_frame = rospy.get_param('~frame')
    # This variable gets the reference frame from parameter 'ref'
    reference_frame = rospy.get_param('~ref')
    # This variable gets the topic name from parameter 'publish_to'
    topic_name = rospy.get_param('~publish_to')
    # This variable gets the human index detected by the Kinect from parameter 'ind'
    index = rospy.get_param('~ind')

    # Initialize the TransformListener TF class
    listener = tf.TransformListener()

    # Initialize the target topic
    tip_pose = rospy.Publisher(topic_name, Pose, queue_size = 10)

    # Setting the data rate as 40 Hz
    rate = rospy.Rate(40.0)

    while not rospy.is_shutdown():
        # Listen to the target frame and doing a transformation with respect to the reference frame
        try:
	    now = rospy.Time.now()
            (trans,rot) = listener.lookupTransform(reference_frame+'_'+str(index), active_frame+'_'+str(index), rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Assigning the transformation data (position and orientation) to a geometry_msgs::Pose message
        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

        # Publishing
        tip_pose.publish(pose)

        # Sleep until 1/40 second
        rate.sleep()
