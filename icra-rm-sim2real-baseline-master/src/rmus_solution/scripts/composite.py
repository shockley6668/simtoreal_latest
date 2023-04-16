#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
class PID_controller:
    def __init__(self,kp,ki,kd,threshold):
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.threshold=threshold
        self.error_sum=0
        self.last_error=0
    def update(self,error,dt=0.01):
        self.error_sum += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        out=self.kp * error + self.ki * self.error_sum + self.kd * derivative
        if out>self.threshold:
            out=self.threshold
        elif out<-self.threshold:
            out=-self.threshold
        return out

class Stop_behaviour(py_trees.behaviour.Behaviour):
    def __init__(self,name="Stop"):
        super().__init__(name)
    
    def update(self):
        return py_trees.Status.RUNNING
class detect_block(py_trees.behaviour.Behaviour):
    def __init__(self,name="detect_block",topic_name="/see_blockid", type=UInt8MultiArray):
        super().__init__(name)
        self.topic_name=topic_name
        self.topic_type=type
        self.subscriber=None
        self.data=[]
        self.blackboard=py_trees.blackboard.Blackboard()
        self.last_data=[]
        
    def initialise(self):
        self.subscriber=rospy.Subscriber(self.topic_name,self.topic_type,self.topic_callback)
        self.begin_time=rospy.get_time()
    def update(self):
        # print(self.data)
        if (rospy.get_time()-self.begin_time)<1:
            return py_trees.Status.RUNNING
        if self.data ==[]:
            self.last_data=self.data      
            return py_trees.Status.RUNNING
        if self.last_data==[]:
            self.last_data=self.data
            return py_trees.Status.RUNNING
        
        # print(self.data)
        placed_block=self.blackboard.get("placed_block")
        if placed_block is not None:
            for i in placed_block:
                if i in self.data:
                    return py_trees.Status.RUNNING
        need_block=self.blackboard.get("need_block")
        # need_block=[1,2,3,4,5]
        lendata=min(len(self.data),len(self.last_data))
        print(self.data,self.last_data)
        
        for i in range(0,lendata):
            if self.data[i]==self.last_data[i]:
                if self.data[i] in need_block:
                    
                    self.blackboard.set("now_take_block",self.data[i])
                    return py_trees.Status.SUCCESS
                else :
                    self.last_data=self.data
                    return py_trees.Status.RUNNING
            else :
                self.last_data=self.data
                return py_trees.Status.RUNNING
            
            
    def topic_callback(self,msg):
        self.data=np.frombuffer(msg.data,np.uint8)
        self.data=self.data.tolist()
class detect_block_early_test(detect_block):
    def __init__(self,name="detect_block_early"):
        super().__init__(name)
    def initialise(self):
        
        # self.odom_pose=Odometry().pose
        # self.odom_pose.pose.position.x=0
        # self.odom_pose.pose.position.y=0
        # self.odom_pose.pose.position.z=0
        # self.odom_pose.pose.orientation.w=-1
        self.subscriber=rospy.Subscriber(self.topic_name,self.topic_type,self.topic_callback)
       
    

    def update(self):
        if self.data==[]:
            return py_trees.Status.RUNNING
        else :
            visilise_place=self.blackboard.get("visilise_place")
            if visilise_place != None:
                for i in self.data:
                    if i not in visilise_place:
                        visilise_place[i]=self.odom_pose
                        
                        self.blackboard.set("visilise_place",visilise_place)
            else :
                visilise_place={}
                while self.odom_pose is None:
                    continue
                for i in self.data:
                    visilise_place[i]=self.odom_pose
                
                self.blackboard.set("visilise_place",visilise_place)
            return py_trees.Status.RUNNING
class detect_block_early(detect_block):
    def __init__(self,name="detect_block_early"):
        super().__init__(name)
    def initialise(self):
        self.odom_sub=rospy.Subscriber("/ep/odom",Odometry,self.get_odom,queue_size=1)
        self.odom_pose=None
        super().initialise()
    
    
    def get_odom(self,msg):
        self.odom_pose=msg.pose
        self.odom_pose.pose.position.y-=0.05
        self.odom_pose.pose.position.x-=0.05
    def update(self):
        if self.data==[]:
            return py_trees.Status.RUNNING
        else :
            visilise_place=self.blackboard.get("visilise_place")
            if visilise_place != None:
                for i in self.data:
                    if i not in visilise_place:
                        visilise_place[i]=self.odom_pose
                        
                        self.blackboard.set("visilise_place",visilise_place)
            else :
                visilise_place={}
                while self.odom_pose is None:
                    continue
                for i in self.data:
                    visilise_place[i]=self.odom_pose
                
                self.blackboard.set("visilise_place",visilise_place)
            return py_trees.Status.RUNNING
class detect_block_later(detect_block_early):
    def __init__(self):
        super().__init__(name="detect_block_later")
    def update(self):
        need_block=self.blackboard.get("need_block")
        for i in self.data:
            if i in need_block:
                placed_block=self.blackboard.get("placed_block")
                if placed_block is not None:
                    for j in placed_block:
                        if j==i:
                            return py_trees.Status.RUNNING
        visilise_place=self.blackboard.get("visilise_place")
        if visilise_place != None:
            for i in self.data:
                if i not in visilise_place:
                    visilise_place[i]=self.odom_pose
                    
                    self.blackboard.set("visilise_place",visilise_place)
        else :
            visilise_place={}
            while self.odom_pose is None:
                continue
            for i in self.data:
                visilise_place[i]=self.odom_pose
            
            self.blackboard.set("visilise_place",visilise_place)
        return py_trees.Status.RUNNING
class arm():
    def __init__(self):
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point, queue_size=2)
    
        self.arm_position_pub = rospy.Publisher("arm_position", Pose, queue_size=2)
    def reset_arm(self):
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.12
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        rospy.loginfo("reset the arm")
        self.arm_position_pub.publish(reset_arm_msg)
    
    def close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        rospy.loginfo("close the gripper")
        self.arm_gripper_pub.publish(close_gripper_msg)
    def pre(self):
        rospy.loginfo("<manipulater>: now prepare to grip")
        pose = Pose()
        pose.position.x = 0.21
        pose.position.y = 0.0
        self.arm_position_pub.publish(pose)
    def pre_fang(self):
        rospy.loginfo("<manipulater>: fang")
        pose = Pose()
        pose.position.x = 0.13
        pose.position.y = 0.08
        self.arm_position_pub.publish(pose)
    def open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        rospy.loginfo("open the gripper")
        self.arm_gripper_pub.publish(open_gripper_msg)

    
###################################################################      
class aim_block(py_trees.behaviour.Behaviour):
    def __init__(self,name='aim_block'):
        super().__init__(name)
        self.current_marker_poses = None
        self.image_time_now = 0
        
        self.get_block_id=None
        self.odom_pose=Pose
        
        
    def markerPoseCb(self, msg):
        self.current_marker_poses = msg
        self.image_time_now = rospy.get_time()
    def get_block_id_callback(self,msg):
        self.see_block=list(np.frombuffer(msg.data,np.uint8))
    def get_odom(self,msg):
        self.flag_odom=False
        self.odom_pose=msg.pose
    def sendBaseVel(self, vel):
        twist = Twist()
        twist.linear.z = 0.0
        twist.linear.x = vel[0]
        twist.linear.y = vel[1]
        twist.angular.z = vel[2]
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        self.cmd_vel_puber.publish(twist)
    def getTargetPosInBaseLink(self, pose_in_cam):
        posestamped_in_cam = tf2_geometry_msgs.PoseStamped()
        posestamped_in_cam.header.stamp = rospy.Time.now()
        posestamped_in_cam.header.frame_id = (
            "camera_aligned_depth_to_color_frame_correct"
        )
        posestamped_in_cam.pose = pose_in_cam
        posestamped_in_base = self.tfBuffer.transform(posestamped_in_cam, "base_link")
        
        
        pose_in_base = posestamped_in_base.pose
        pos = np.array(
            [pose_in_base.position.x, pose_in_base.position.y, pose_in_base.position.z]
        )
        return pos
    def is_stop(self,msg):
        self.data_stop=msg
    def setup(self,timeout):
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.action_client=actionlib.SimpleActionClient("/move_base",move_base_msgs.MoveBaseAction)
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            self.logger.error("{0}.setup() could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))
            self.action_client = None
            return False
        return True
    def initialise(self):
        self.aim_begin_time=rospy.get_time()
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point, queue_size=2)
        self.see_block=[]
        
        self.arm_position_pub = rospy.Publisher("arm_position", Pose, queue_size=2)
        self.blackboard=py_trees.blackboard.Blackboard()
        self.flag_odom=True
        self.good_pub=rospy.Publisher('/good_pose',Float64MultiArray,queue_size=1)
        self.cmd_vel_puber = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/see_blockid",UInt8MultiArray,self.get_block_id_callback,queue_size=1)
        rospy.Subscriber("/get_blockinfo", Pose, self.markerPoseCb, queue_size=1)
        # rospy.Subscriber("/ep/odom",Odometry,self.get_odom,queue_size=1)
        rospy.Subscriber('/rplidar/scan', LaserScan, self.laser_callback,queue_size=5)
        self.cmd_pos_pub=rospy.Publisher("/cmd_position",Twist,queue_size=1)
        self.sendBaseVel([0,0,0])
        self.errdet_flag=True
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        try:
            self.switch_mode =rospy.ServiceProxy('/image_processor_switch_mode',switch)
        except rospy.ServiceException as e:
            print("switch mode Service call failed: %s"%e)
        
        f_block=self.blackboard.get("now_take_block")
        self.switch_mode(f_block)
        
        pos_inmap=self.ca_posinmap()
        
        self.goodpose=[0,0,0]
        self.goodpose[0]=pos_inmap[0]+0.32*math.cos(pos_inmap[2]+math.pi)
        self.goodpose[1]=pos_inmap[1]+0.32*math.sin(pos_inmap[2]+math.pi)
        self.goodpose[2]=pos_inmap[2] 
        self.action_client.send_goal(self.creat_goal(self.goodpose[0], self.goodpose[1],self.goodpose[2]))
        self.not_in_times=0
        #vis
        # self.br = tf2_ros.TransformBroadcaster()
        # self.t =TransformStamped()

        # self.t.header.stamp = rospy.Time.now()
        # self.t.header.frame_id = "map"
        # self.t.child_frame_id = "good"
        # self.t.transform.translation.x = self.goodpose[0]
        # self.t.transform.translation.y = self.goodpose[1]
        # self.t.transform.translation.z = 0.0
        # qua=quaternion_from_euler(0,0,self.goodpose[2])
        # self.t.transform.rotation.x= qua[0]
        # self.t.transform.rotation.y =qua[1]
        # self.t.transform.rotation.z = qua[2]
        # self.t.transform.rotation.w = qua[3]

        # self.t.header.stamp = rospy.Time.now()
        # self.br.sendTransform(self.t)
        
        self.lx_control=PID_controller(4,1.2,0.2,0.5)
        self.ly_control=PID_controller(4.1,1.7,0.2,0.5)
        
        
        self.last_time=None
        self.x_flag=False
        self.all_flag=False
        self.lx_flag=False
        self.ly_flag=False

        self.deque=deque()
        self.deque_x=deque()
        self.deque_y=deque()
        self.ranges=None
        self.last_goodpose=None
        self.retry=False
        
    def ca_posinmap(self):
        while self.current_marker_poses==None:
            continue
        # while not self.tfBuffer.can_transform(
        #     "map", "camera_aligned_depth_to_color_frame_correct", rospy.Time.now()
        # ):
        #     continue
        transform = self.tfBuffer.lookup_transform('map', 'camera_aligned_depth_to_color_frame_correct', rospy.Time(0), rospy.Duration(1.0))

        posestamped_in_cam =  tf2_geometry_msgs.PoseStamped()
        posestamped_in_cam.header.stamp = rospy.Time(0)
        posestamped_in_cam.header.frame_id = (
            "camera_aligned_depth_to_color_frame_correct"
        )
        
        posestamped_in_cam.pose = self.current_marker_poses
        
        posestamped_in_map= tf2_geometry_msgs.do_transform_pose(posestamped_in_cam, transform)
        quat = np.array(
                [
                    posestamped_in_map.pose.orientation.x,
                    posestamped_in_map.pose.orientation.y,
                    posestamped_in_map.pose.orientation.z,
                    posestamped_in_map.pose.orientation.w,
                ]
            )
        block_angle=euler_from_quaternion(quat)[2]+math.pi/2
        pos_inmap=[0,0,0]
        pos_inmap[1]=posestamped_in_map.pose.position.y
        pos_inmap[0]=posestamped_in_map.pose.position.x
        pos_inmap[2]=block_angle
        print(pos_inmap)
        return pos_inmap
    def laser_callback(self,msg):
        self.ranges = msg.ranges
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
    def update(self):
        f_block=self.blackboard.get("now_take_block")
        if not self.all_flag:
            if (rospy.get_time()-self.aim_begin_time)>9:
                return py_trees.Status.FAILURE 
            
            # pos_inmap=self.ca_posinmap()        
            # self.goodpose=[0,0,0]
            # self.goodpose[0]=pos_inmap[0]+0.4*math.cos(pos_inmap[2]+math.pi)
            # self.goodpose[1]=pos_inmap[1]+0.4*math.sin(pos_inmap[2]+math.pi)
            # self.goodpose[2]=pos_inmap[2] 
            # self.action_client.send_goal(self.creat_goal(self.goodpose[0], self.goodpose[1],self.goodpose[2]))
            result = self.action_client.get_result()
            state=self.action_client.get_state()
            print(rospy.loginfo('Client state: %s', state))
            if result: 
                self.all_flag=True
                print("move base aim success")
                return py_trees.Status.RUNNING
            if (state==4 or state==0) and not self.all_flag:
                pos_inmap=self.ca_posinmap()
                self.goodpose=[0,0,0]
                self.goodpose[0]=pos_inmap[0]+0.32*math.cos(pos_inmap[2]+math.pi)
                self.goodpose[1]=pos_inmap[1]+0.32*math.sin(pos_inmap[2]+math.pi)
                self.goodpose[2]=pos_inmap[2] 
                self.action_client.send_goal(self.creat_goal(self.goodpose[0], self.goodpose[1],self.goodpose[2]))
                
            
            # if self.action_client.get_state in  [GoalStatus.ABORTED,GoalStatus.PREEMPTED]:
            #     return py_trees.Status.FAILURE
            # left_distance = min(self.ranges[0: len(self.ranges) // 4])
            # right_distance = min(self.ranges[3 * len(self.ranges) // 4: len(self.ranges)])
            

            # if left_distance < 0.2:
            #     self.sendBaseVel([0,0.2,0])
            #     now_place_inMB=self.blackboard.get("places_inMB")
            #     index=self.blackboard.get("places_index")
            #     now_place_inMB[index][0]=False
            #     self.blackboard.set("places_inMB",now_place_inMB)
            #     rospy.sleep(0.8)
            #     self.sendBaseVel([0,0,0])

            #     return py_trees.Status.FAILURE
            # if right_distance < 0.2:
            #     self.sendBaseVel([0,0.2,0])
            #     now_place_inMB=self.blackboard.get("places_inMB")
            #     index=self.blackboard.get("places_index")
            #     now_place_inMB[index][0]=False
            #     self.blackboard.set("places_inMB",now_place_inMB)
            #     rospy.sleep(0.8)
            #     self.sendBaseVel([0,0,0])

            #     return py_trees.Status.FAILURE 
        else:
            # self.sendBaseVel([0,0,0])
            if f_block not in self.see_block:
                self.not_in_times+=1
            if self.not_in_times>8:
                if self.retry:
                    return py_trees.Status.FAILURE
                else:
                    self.sendBaseVel([-0.8,0,0])
                    rospy.sleep(0.5)
                    self.sendBaseVel([0,0,0])
                    self.retry=True
                    return py_trees.Status.RUNNING
                
            cmd_vel=[0,0,0]
            while self.current_marker_poses==None:
                continue
            # need_block=np.frombuffer(self.uint8_need_b,np.uint8)
            # if need_block==[]:
            #     return py_trees.Status.FAILURE
            pos=self.getTargetPosInBaseLink(self.current_marker_poses)
            pos=list(pos)
            # cmd_vel[0]=self.lx_control.update(pos[0]-0.385,rospy.get_time()-self.last_time)
            
           
            if abs(pos[1])<=0.01:
                self.ly_flag=True
                cmd_vel[1]=0
                
            if self.ly_flag:
                if not self.lx_flag:

                    cmd_vel[0]=self.lx_control.update(pos[0]-0.38,rospy.get_time()-self.last_time)
            else:
                cmd_vel[1]=self.ly_control.update(pos[1],rospy.get_time()-self.last_time)
            if abs(pos[0]-0.38)<0.02 :
                self.lx_flag=True
                cmd_vel[0]=0
            if self.lx_flag and self.ly_flag:
                self.sendBaseVel([0,0,0])
                twist = Twist()
                twist.linear.x = 0.1
                self.cmd_pos_pub.publish(twist)
                rospy.sleep(1)
                return py_trees.Status.SUCCESS
            else:
                print(cmd_vel)
                self.sendBaseVel(cmd_vel)
        self.last_time=rospy.get_time()
        return py_trees.Status.RUNNING

class take_blcok(py_trees.behaviour.Behaviour):
    def __init__(self,name="take_block"):
        super().__init__(name)
    def close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        rospy.loginfo("close the gripper")
        self.arm_gripper_pub.publish(close_gripper_msg)
    def open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        rospy.loginfo("open the gripper")
        self.arm_gripper_pub.publish(open_gripper_msg)
    def reset_arm(self):
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.12
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        rospy.loginfo("reset the arm")
        self.arm_position_pub.publish(reset_arm_msg)   
    def initialise(self):
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point, queue_size=2)
    
        self.arm_position_pub = rospy.Publisher("arm_position", Pose, queue_size=2)
    
        self.cmd_vel_puber = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    def update(self):
        pose = Pose()
        pose.position.x = 0.19
        pose.position.y = -0.08
        self.arm_position_pub.publish(pose)
        rospy.sleep(1.0)
        rospy.loginfo("Place: reach the goal for placing.")
        self.close_gripper()
        rospy.sleep(1.0)

        self.close_gripper()
        rospy.sleep(1.0)
        self.reset_arm()
        return py_trees.Status.SUCCESS


class see_need_block(py_trees.behaviour.Behaviour):
    def __init__(self,name="see_need_block"):
        super().__init__(name)
        
    def need_block_callback(self,msg):
        self.need_block=np.frombuffer(msg.data,np.uint8)
        self.need_block=self.need_block.tolist()
    def initialise(self):
        self.need_block=[]
        try:
            self.switch_mode =rospy.ServiceProxy('/image_processor_switch_mode',switch)
            self.switch_mode.call(9)
        except rospy.ServiceException as e:
            print("switch mode Service call failed: %s"%e)
        self.see=rospy.Subscriber("/get_gameinfo",UInt8MultiArray,self.need_block_callback,queue_size=1)
        self.blackboard=py_trees.blackboard.Blackboard()
        
    def update(self):
        if self.need_block==[]:

            return py_trees.Status.RUNNING
        else :
            self.blackboard.set("need_block",self.need_block)
            self.switch_mode.call(1)
            return py_trees.Status.SUCCESS

class place_block(py_trees.behaviour.Behaviour):
    def __init__(self,name="palce_block"):
        super().__init__(name)
        self.current_marker_poses=None
        self.ep_arm=arm()
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
    def markerPoseCb(self, msg):
        self.current_marker_poses = msg
    def sendBaseVel(self, vel):
        twist = Twist()
        twist.linear.z = 0.0
        twist.linear.x = vel[0]
        twist.linear.y = vel[1]
        twist.angular.z = vel[2]
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        self.cmd_vel_puber.publish(twist)
    def getTargetPosInBaseLink(self, pose_in_cam):
        posestamped_in_cam = tf2_geometry_msgs.PoseStamped()
        posestamped_in_cam.header.stamp = rospy.Time.now()
        posestamped_in_cam.header.frame_id = (
            "camera_aligned_depth_to_color_frame_correct"
        )
        posestamped_in_cam.pose = pose_in_cam
        posestamped_in_base = self.tfBuffer.transform(posestamped_in_cam, "base_link")
        
        
        pose_in_base = posestamped_in_base.pose
        pos = np.array(
            [pose_in_base.position.x, pose_in_base.position.y, pose_in_base.position.z]
        )
        return pos   
    def placed_block_id_callback(self,msg):
        self.placed_block_idlist=list(np.frombuffer(msg.data,np.uint8))
    def initialise(self):
        self.blackboard=py_trees.blackboard.Blackboard()
        now_take_block=self.blackboard.get("now_take_block")
        need_block=self.blackboard.get("need_block")
        
        
        # need_block=[5,3,4]
        self.switch_mode=rospy.ServiceProxy('/image_processor_switch_mode',switch)
        self.cmd_vel_puber = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/get_blockinfo", Pose, self.markerPoseCb, queue_size=1)
        self.cmd_pos_pub=rospy.Publisher("/cmd_position",Twist,queue_size=1)
        
        self.switch_mode.call(0)
        print(now_take_block)
        if now_take_block !=0:
            self.switch_mode.call(need_block.index(now_take_block)+6)

        rospy.Subscriber("/placed_blockid",UInt8MultiArray,self.placed_block_id_callback,queue_size=1)
        self.y_control=PID_controller(3.6,0.09,0.2,0.5)
        self.last_time=None
        self.aim=False
    def update(self):
        if not self.aim:
            if self.current_marker_poses is None:
                self.last_time =rospy.get_time()
                return py_trees.Status.RUNNING
            else :
                if self.last_time==None:
                    self.last_time=rospy.get_time()
                    return py_trees.Status.RUNNING
                pos=self.getTargetPosInBaseLink(self.current_marker_poses)
                cmd_vel=[0,0,0]
                if abs(pos[1])>0.018:
                    cmd_vel[1]=self.y_control.update(pos[1],rospy.get_time()-self.last_time)
                    self.sendBaseVel(cmd_vel)
                    return py_trees.Status.RUNNING
                self.sendBaseVel([0,0,0])
                self.aim=True
        self.ep_arm.pre()
        twist=Twist()
        twist.linear.x = 0.2
        self.cmd_pos_pub.publish(twist)
        rospy.sleep(3)
        twist.linear.x = 0
        self.cmd_pos_pub.publish(twist)
        rospy.sleep(1.0)
        self.ep_arm.open_gripper()
        rospy.sleep(1.0)
        reset_thread = threading.Thread(target=self.ep_arm.reset_arm)
        reset_thread.start()
        twist.linear.x = -0.3
        self.cmd_pos_pub.publish(twist)
        self.switch_mode.call(10)
        rospy.sleep(0.7)
        twist.linear.x = 0
        self.cmd_pos_pub.publish(twist)

        now_take_block=self.blackboard.get("now_take_block")
        if now_take_block in self.placed_block_idlist: 
            placed_block=self.blackboard.get("placed_block")
            if placed_block==None:
                self.blackboard.set("placed_block",[now_take_block])
            else:
                placed_block.append(now_take_block)
                self.blackboard.set("placed_block",placed_block)
            place_block_time=self.blackboard.get("place_block_times")
            if place_block_time==None:
                self.blackboard.set("place_block_times",1)
            else :
                self.blackboard.set("place_block_times",place_block_time+1)
                print(place_block_time)
        else:
            now_place_inMB=self.blackboard.get("places_inMB")
            index=self.blackboard.get("places_index")
            now_place_inMB[index][0]=False
            self.blackboard.set("places_inMB",now_place_inMB)
            self.switch_mode.call(1)
            return py_trees.Status.FAILURE
        self.switch_mode.call(1)
        return py_trees.Status.SUCCESS

class check_all_block(py_trees.behaviour.Behaviour):
    def __init__(self) -> None:
        super().__init__("check_all_block")
        self.blackboard=py_trees.blackboard.Blackboard()
    def update(self):
        place_time=self.blackboard.get("place_block_times")
        if place_time == None or place_time < 3 :
            
            return py_trees.Status.RUNNING 
        else :
            print(place_time)
            return py_trees.Status.SUCCESS

class check_saw(py_trees.behaviour.Behaviour):
    def __init__(self) -> None:
        super().__init__(name="check_saw")
    def initialise(self):
        self.blackboard=py_trees.blackboard.Blackboard()
        self.need_block=self.blackboard.get("need_block")
        self.visilise_place=self.blackboard.get("visilise_place")
        self.index=0
    def update(self):
        print(self.visilise_place)
        if self.visilise_place == None or self.visilise_place=={}:
            return py_trees.Status.SUCCESS
        else:

            key,value=self.visilise_place.popitem()
            if value==None:
                return py_trees.Status.RUNNING
            placed_block=self.blackboard.get("placed_block")
            print(placed_block)
            if placed_block is not None:
                if key in placed_block:
                    return py_trees.Status.RUNNING
            if key in self.need_block:
                self.blackboard.set("goto_visi_place",value)
                print('y')
                return py_trees.Status.FAILURE
            
        return py_trees.Status.RUNNING
    