#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32
bridge = CvBridge()
camera_info = None

def camera_info_callback(data):
    global camera_info
    camera_info = data
def callback(data):
    global camera_info
    if camera_info is None:
        return

    depth_image = bridge.imgmsg_to_cv2(data, "passthrough")
    depth_array = np.array(depth_image, dtype=np.float32)
    depth_array[depth_array < 0] = 0

    # 只取图像中心位置的深度值
    center_x = int(depth_array.shape[1] / 2)
    center_y = int(depth_array.shape[0] / 2)
    center_distance = depth_array[center_y, center_x]

    # 发布计算结果
    pub.publish(center_distance)
    rospy.loginfo("The distance to the obstacle in the center of the image is %f meters." % center_distance)

def main():
    rospy.init_node('center_depth_distance_publisher', anonymous=True)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback)
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_info_callback)
    pub=rospy.Publisher("center_distances",Float32,queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()

