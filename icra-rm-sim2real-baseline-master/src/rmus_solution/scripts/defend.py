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
from composite import aim_block ,Stop_behaviour,take_blcok,place_block,see_need_block,detect_block,detect_block_early,detect_block_later,check_saw,check_all_block,detect_block_early_test
import move_base_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs
import defend_behav
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt8MultiArray
from aim_easy import aim_block_easy
def create_root():
    # test=action_behavior("m1",creat_goal(1,1,0),"/move_base")
    det=detect_block()

    root=py_trees.composites.Sequence("root")
    
    goto_watch_board=py_trees.composites.Sequence("goto_watch_board")
    # goto_watch_board_action=py_trees.composites.Parallel("goto_watch_board_action",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    see_need_block_=see_need_block()
    find_three_and_check=py_trees.composites.Parallel("find_three_and_check",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

    find_three=py_trees.Sequence("start_find_three")
    

    start_find=py_trees.composites.Parallel("start_find",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    travel=py_trees.composites.Parallel("travel",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
    start_travel=py_trees.composites.Sequence("start_travel")
    check_place_=check_place(goal_places=[[0.00, 1.60, -1.5,-0.5],[1, 1.60, -1.5,-13],[0.8, 0.2,1.5,11],[0.9,-0.7,0,-0.5],[2.6,-0.7,0,12],[2.5,0,-3,-6],[2.5,1,-3,-6],[2.3,2.3,0.8,7],[1.5,2.8,1.5,6],[0.9,3.2,3,5],[0.05,3,-1.5,0.1]])

    goto_place_=goto_place()
    rotate_=rotate()
    check_all_place=py_trees.behaviours.Running(name='check_all_place')
    stop=Stop_behaviour()
    aim=aim_block()
    # aim=aim_block_easy()
    take=take_blcok()
    goto_defence_palce=defend_behav.goto_block_place()
    goto_watch_board_behav=action_behavior(action_goal=(creat_goal(-0.05, 1.60, 0.00)),name="goto_watch_board_behav")
    defend_placed=py_trees.composites.Sequence("defend_placed")
    ##write your position that placed  here

    defend_check=defend_behav.check_placed_place(goal_places=[[0.1,2.6,-0.5],[0.2,2.9,1],[1.5,1,-0.5]])
    
    defend_placed.add_children([defend_check,goto_defence_palce])
    place=defend_behav.defend_placed()
    check_all_block_=check_all_block()
    running=py_trees.behaviours.Running(name='runing')
    
    #trees

    start_travel.add_children([check_place_,goto_place_,rotate_])
    travel.add_children([start_travel,check_all_place])
    start_find.add_children([det,travel])
    # visilise.add_children([check_saw_,goto_visi_place_])


    find_three.add_children([start_find,aim,take,defend_placed,place])

    
    find_three_and_check.add_children([find_three,check_all_block_])
    goto_watch_board.add_children([goto_watch_board_behav,see_need_block_])
    root.add_children([goto_watch_board,find_three_and_check,stop])
    
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
    
