#!/usr/bin/env python
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
        topic_name = '/euler_gripper'
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


    # def RtoQuat(self, T_matrix):
    #     trace = T_matrix[0,0] + T_matrix[1,1] + T_matrix[2,2]
    #     if( trace > 0 ):
    #         s = 0.5 / sqrt(trace+ 1.0);
    #         w = 0.25 / s;
    #         x = ( T_matrix[2,1] - T_matrix[1,2] ) * s;
    #         y = ( T_matrix[0,2] - T_matrix[2,0] ) * s;
    #         z = ( T_matrix[1,0] - T_matrix[0,1] ) * s;
    #     else:
    #         if ( T_matrix[0,0] > T_matrix[1,1] and T_matrix[0,0] > T_matrix[2,2] ):
    #             s = 2.0 * sqrt( 1.0 + T_matrix[0,0] - T_matrix[1,1] - T_matrix[2,2]);
    #             w = (T_matrix[2,1] - T_matrix[1,2] ) / s
    #             x = 0.25 * s
    #             y = (T_matrix[0,1] + T_matrix[1,0] ) / s
    #             z = (T_matrix[0,2] + T_matrix[2,0] ) / s;
    #         elif (T_matrix[1,1] > T_matrix[2,2]):
    #             s = 2.0 * sqrt( 1.0 + T_matrix[1,1] - T_matrix[0,0] - T_matrix[2,2]);
    #             w = (T_matrix[0,2] - T_matrix[2,0] ) / s;
    #             x = (T_matrix[0,1] + T_matrix[1,0] ) / s;
    #             y = 0.25 * s;
    #             z = (T_matrix[1,2] + T_matrix[2,1] ) / s;
    #         else:
    #             s = 2.0 * sqrt( 1.0 + T_matrix[2,2] - T_matrix[0,0] - T_matrix[1,1] );
    #             w = (T_matrix[1,0] - T_matrix[0,1] ) / s;
    #             x = (T_matrix[0,2] + T_matrix[2,0] ) / s;
    #             y = (T_matrix[1,2] + T_matrix[2,1] ) / s;
    #             z = 0.25 * s;
    #     self.quat = Matrix([[x],[y],[z],[w]])

if __name__ == '__main__':
    try:
        listen_orientation()
    except rospy.ROSInterruptException: pass
