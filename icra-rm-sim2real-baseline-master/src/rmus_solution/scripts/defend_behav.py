import py_trees

from actionlib_msgs.msg import GoalStatus
import rospy
from std_msgs.msg import UInt8MultiArray ,Float64MultiArray
import sys
import math
import actionlib
import move_base_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped,TransformStamped
from rmus_solution.srv import switch , graspsignal
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import threading
import csv
from collections import deque
from sensor_msgs.msg import LaserScan
from travel import action_behavior
from composite import arm
class check_placed_place(py_trees.behaviour.Behaviour):

    def __init__(self,name="check_palced_palce",goal_places=[]):
        super().__init__(name)
 
        places_inMB=[]
        self.blackboard=py_trees.blackboard.Blackboard()
        for goal_place in goal_places:
            places_inMB.append([False,self.creat_goal(goal_place[0],goal_place[1],goal_place[2])])
        self.blackboard.set("block_places_inMB",places_inMB)       
    def creat_goal(self,x,y,yaw):
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
    def initialise(self):
        self.index=0
    #     print("init")
    def update(self):
        now_place_inMB=self.blackboard.get("block_places_inMB")
        if now_place_inMB[self.index][0]==False:
            self.blackboard.set("next_block_goal",now_place_inMB[self.index][1])
            now_place_inMB[self.index][0]=True
            self.blackboard.set("block_places_inMB",now_place_inMB)
            self.blackboard.set("block_places_index",self.index) 
            return py_trees.Status.SUCCESS
        else:
            
            if self.index<len(now_place_inMB)-1:
                self.index+=1
                return py_trees.Status.RUNNING
            else :
                self.index=0
                for i in range(0,len(now_place_inMB)):
                    now_place_inMB[i][0]=False
                self.blackboard.set("block_places_inMB",now_place_inMB) 
                return py_trees.Status.RUNNING
class goto_block_place(action_behavior):
    def __init__(self):
        super().__init__(name="goto_block_place",action_namespace="/move_base",override_feedback_message_on_running="moving")
    def initialise(self):
        blackboard=py_trees.blackboard.Blackboard()
        self.action_goal=blackboard.get("next_block_goal")
        
        super().initialise()
class defend_placed(py_trees.behaviour.Behaviour):
    def __init__(self,name="defend_placed") -> None:
        super().__init__(name)
    def initialise(self):
        self.ep_arm=arm()
        self.blackboard=py_trees.blackboard.Blackboard()
    def update(self):
        self.ep_arm.open_gripper()
        rospy.sleep(1.0)
        place_block_time=self.blackboard.get("place_block_times")
        reset_thread = threading.Thread(target=self.ep_arm.reset_arm)
        reset_thread.start()
        placed_block=self.blackboard.get("placed_block")
        now_take_block=self.blackboard.get("now_take_block")
        if placed_block==None:
            self.blackboard.set("placed_block",[now_take_block])
        else:
            placed_block.append(now_take_block)
            self.blackboard.set("placed_block",placed_block)
        if place_block_time==None:
            self.blackboard.set("place_block_times",1)
        else :
            self.blackboard.set("place_block_times",place_block_time+1)
            print(place_block_time)
        return py_trees.Status.SUCCESS



    