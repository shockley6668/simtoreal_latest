#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion

if __name__ == '__main__':
    rospy.init_node('action_goal_publisher')

   
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()

  
    goal = MoveBaseGoal()

    # Set the goal target position (x, y, z) and orientation (quaternion)
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(0.0, -1.0, 0.0)
    goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0) 

    client.send_goal(goal)

  
    while not client.get_result():
        if client.get_state()!=1 and client.get_state()!=0:
            rospy.loginfo('Client state: %s', client.get_state())

    

    rospy.loginfo(client.get_result())