#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv
import rospy
from std_msgs.msg import Float64MultiArray,Bool
f = open('csv_file.csv', 'w')
# create the csv writer
writer = csv.writer(f)
# writer.writerow([111])
# f.close()
# write a row to the csv file

def data_call_back(msg):
    global data_fa
    data_fa=msg.data
    data_fa=list(data_fa)
    print(data_fa)
    writer.writerow(data_fa)
def stop_call_back(msg):
    global is_stop
    is_stop=msg
    f.close()
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/good_pose", Float64MultiArray, data_call_back)
rospy.Subscriber("/is_stop", Bool, stop_call_back)
rospy.spin()

