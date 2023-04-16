#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import functools
import py_trees
import py_trees_ros
import py_trees.console as console
from actionlib_msgs.msg import GoalStatus
import rospy
import sys
from math import pi
from travel import action_behavior,check_place,goto_place,rotate,goto_visi_place
from composite import aim_block,Stop_behaviour,take_blcok,place_block,see_need_block,detect_block,detect_block_early,detect_block_later,check_saw,check_all_block,detect_block_early_test
import move_base_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt8MultiArray
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped,TransformStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import math
from rmus_solution.srv import switch
from nav_msgs.msg import Odometry
from aim_easy import aim_block_easy
def create_root():
    # test=action_behavior("m1",creat_goal(1,1,0),"/move_base")
    root=py_trees.composites.Sequence("root")
    aim=aim_block()
    det=detect_block()
    stop=Stop_behaviour()
    rotate_=rotate()
    root.add_children([det,aim,stop])
    
    # travel.add_children([aim])

    return root
def creat_goal(x,y,yaw):
    goal_pose=move_base_msgs.MoveBaseGoal()
    # 将上面的Euler angles转换成Quaternion的格式
    q_angle=quaternion_from_euler(0, 0, yaw,axes='sxyz')
    q=Quaternion(*q_angle)
    goal_pose.target_pose.header.frame_id='map'
    goal_pose.target_pose.pose.position.x=x
    goal_pose.target_pose.pose.position.y=y
    goal_pose.target_pose.pose.position.z=0
    goal_pose.target_pose.pose.orientation=q
    return goal_pose

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()
    
if __name__=="__main__":
    rospy.init_node("tree")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(60)
    # client=actionlib.SimpleActionClient('move_base',move_base_msgs.MoveBaseAction)
    # client.wait_for_server()
    # client.send_goal(creat_goal(0,0,0))
    # client.wait_for_result()
    # if client.get_state()==GoalStatus.SUCCEEDED:
    #     rospy.loginfo("gaol_reach")
    # else :
    #     rospy.logerr("fuck")
    
