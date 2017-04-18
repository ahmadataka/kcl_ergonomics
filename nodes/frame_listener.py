#!/usr/bin/env python
import roslib
roslib.load_manifest('kcl_ergonomics')
import rospy
import tf
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('frame_listener')

    active_frame = rospy.get_param('~frame')
    reference_frame = rospy.get_param('~ref')
    topic_name = rospy.get_param('~publish_to')
    index = rospy.get_param('~ind')
    listener = tf.TransformListener()

    tip_pose = rospy.Publisher(topic_name, Pose, queue_size = 10)
    rate = rospy.Rate(40.0)
    while not rospy.is_shutdown():
	#(trans,rot) = listener.lookupTransform('openni_depth_frame', 'openni_depth_optical_frame', rospy.Time(0))

        try:
	    now = rospy.Time.now()
            (trans,rot) = listener.lookupTransform(reference_frame+'_'+str(index), active_frame+'_'+str(index), rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]
        tip_pose.publish(pose)

        rate.sleep()
