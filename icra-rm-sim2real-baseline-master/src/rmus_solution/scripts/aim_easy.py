#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import py_trees
from composite import PID_controller
from std_msgs.msg import UInt8MultiArray 
from geometry_msgs.msg import Pose, Point, Twist
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from rmus_solution.srv import switch 
from tf.transformations import euler_from_quaternion
import math
class aim_block_easy(py_trees.behaviour.Behaviour):
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
        quat = np.array(
                [
                    pose_in_base.orientation.x,
                    pose_in_base.orientation.y,
                    pose_in_base.orientation.z,
                    pose_in_base.orientation.w,
                ]
            )
        
        pos =[pose_in_base.position.x, pose_in_base.position.y,euler_from_quaternion(quat)[2]+math.pi/2]
        
        return pos
    def is_stop(self,msg):
        self.data_stop=msg
     
    def initialise(self):
        self.aim_begin_time=rospy.get_time()
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point, queue_size=2)
        self.see_block=[]
        
        self.arm_position_pub = rospy.Publisher("arm_position", Pose, queue_size=2)
        self.blackboard=py_trees.blackboard.Blackboard()
        self.flag_odom=True
    
        self.cmd_vel_puber = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/see_blockid",UInt8MultiArray,self.get_block_id_callback,queue_size=1)
        rospy.Subscriber("/get_blockinfo", Pose, self.markerPoseCb, queue_size=1)
        # rospy.Subscriber("/ep/odom",Odometry,self.get_odom,queue_size=1)
        # rospy.Subscriber('/rplidar/scan', LaserScan, self.laser_callback,queue_size=5)
        self.cmd_pos_pub=rospy.Publisher("/cmd_position",Twist,queue_size=1)
        self.sendBaseVel([0,0,0])
        self.errdet_flag=True
        ##have some problem
        
        
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        try:
            self.switch_mode =rospy.ServiceProxy('/image_processor_switch_mode',switch)
        except rospy.ServiceException as e:
            print("switch mode Service call failed: %s"%e)
        
        f_block=self.blackboard.get("now_take_block")
        self.switch_mode(f_block)
        
        
        
        self.lx_control=PID_controller(3,0.9,0.2,0.2)
        self.ly_control=PID_controller(3,0.9,0.2,0.5)
        self.lw_control=PID_controller(3,0.5,0.2,0.5)
        
        self.last_time=None
        self.x_flag=False
        self.all_flag=False
        self.lx_flag=False
        self.ly_flag=False
        self.lw_flag=False
        self.ranges=None
        self.last_goodpose=None
        
    def laser_callback(self,msg):
        self.ranges = msg.ranges
 
    def update(self):
        f_block=self.blackboard.get("now_take_block")
        if self.last_time==None:
            self.last_time=rospy.get_time()
            return py_trees.Status.RUNNING
        if (rospy.get_time()-self.aim_begin_time)>15:
            return py_trees.Status.FAILURE 
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
        
        if f_block not in self.see_block:
            self.sendBaseVel([0,0,0])
            return py_trees.Status.FAILURE
        cmd_vel=[0,0,0]
        if self.current_marker_poses==None:
            return py_trees.Status.RUNNING
        # need_block=np.frombuffer(self.uint8_need_b,np.uint8)
        # if need_block==[]:
        #     return py_trees.Status.FAILURE
        pos=self.getTargetPosInBaseLink(self.current_marker_poses)
        print(pos)
        
        if self.lx_flag and self.ly_flag:
            self.sendBaseVel([0,0,0])
            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_pos_pub.publish(twist)
            rospy.sleep(1)
            return py_trees.Status.SUCCESS
        if abs(pos[1])<=0.01:
            self.ly_flag=True
            cmd_vel[1]=0
            
        if self.ly_flag and not self.lx_flag:
            cmd_vel[0]=self.lx_control.update(pos[0]-0.385,rospy.get_time()-self.last_time)
            
        if not self.ly_flag:
            cmd_vel[1]=self.ly_control.update(pos[1],rospy.get_time()-self.last_time)
        if abs(pos[0]-0.4)<0.02 :
            self.lx_flag=True
            cmd_vel[0]=0


        
        
        
        else:
            print(cmd_vel)
            self.sendBaseVel(cmd_vel)
        self.last_time=rospy.get_time()
        return py_trees.Status.RUNNING