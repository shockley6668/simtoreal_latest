#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf2_geometry_msgs
import math
from geometry_msgs.msg import PoseStamped
import tf
from nav_msgs.msg import Odometry
if __name__ == '__main__':
    rospy.init_node('baselink_to_map')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    carto_odom_pub=rospy.Publisher("carto_odom", Odometry, queue_size=10)
    rate = rospy.Rate(30.0)

    while not rospy.is_shutdown():
        try:
            # Get the transform from baselink to map
            transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))

            # Transform a pose from baselink to map
            pose_in = PoseStamped()
            pose_in.header.frame_id = 'base_link'
            pose_in.header.stamp = rospy.Time(0)
            pose_in.pose.position.x = 0.0
            pose_in.pose.position.y = 0.0
            pose_in.pose.position.z = 0.0
            pose_in.pose.orientation = transform.transform.rotation
            pose_out = tf2_geometry_msgs.do_transform_pose(pose_in, transform)

            # Convert quaternion to Euler angles
            roll, pitch, yaw = tf.transformations.euler_from_quaternion([pose_out.pose.orientation.x,
                                                                          pose_out.pose.orientation.y,
                                                                          pose_out.pose.orientation.z,
                                                                          pose_out.pose.orientation.w])
            # Do something with the transform and transformed pose
            # For example, print the position and orientation of baselink in map frame
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'map'
            odom.child_frame_id = 'baselink'
            odom.pose.pose.position = pose_out.pose.position
            odom.pose.pose.orientation = pose_out.pose.orientation

            carto_odom_pub.publish(odom)

            rospy.loginfo("odom(%f, %f, %f)",
                          pose_out.pose.position.x, pose_out.pose.position.y, math.degrees(yaw))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        rate.sleep()