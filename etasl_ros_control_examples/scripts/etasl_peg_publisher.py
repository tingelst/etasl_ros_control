#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import numpy as np

import rospy
from std_msgs.msg import Float64, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

peg_etasl_1 = rospy.Publisher(
    '/etasl_controller_1/peg_frame', Pose, queue_size=3)
peg_etasl_2 = rospy.Publisher(
    '/etasl_controller_2/peg_frame', Pose, queue_size=3)

def peg1_frame_clbk(data):
    peg1_frame = data.pose 
    if (rospy.get_param('pegNdx') == 1):
        peg_etasl_1.publish(peg1_frame)
        peg_etasl_2.publish(peg1_frame)

def peg2_frame_clbk(data):
    peg2_frame = data.pose 
    if (rospy.get_param('pegNdx') == 2):
        peg_etasl_1.publish(peg2_frame)
        peg_etasl_2.publish(peg2_frame)

def peg3_frame_clbk(data):
    peg3_frame = data.pose 
    if (rospy.get_param('pegNdx') == 3):
        peg_etasl_1.publish(peg3_frame)
        peg_etasl_2.publish(peg3_frame)

def peg4_frame_clbk(data):
    peg4_frame = data.pose 
    if (rospy.get_param('pegNdx') == 4):
        peg_etasl_1.publish(peg4_frame)
        peg_etasl_2.publish(peg4_frame)

def peg5_frame_clbk(data):
    peg5_frame = data.pose 
    if (rospy.get_param('pegNdx') == 5):
        peg_etasl_1.publish(peg5_frame)
        peg_etasl_2.publish(peg5_frame)

def listener():
    rospy.Subscriber('/rviz/peg1_frame', PoseStamped, peg1_frame_clbk)
    rospy.Subscriber('/rviz/peg2_frame', PoseStamped, peg2_frame_clbk)
    rospy.Subscriber('/rviz/peg3_frame', PoseStamped, peg3_frame_clbk)
    rospy.Subscriber('/rviz/peg4_frame', PoseStamped, peg4_frame_clbk)
    rospy.Subscriber('/rviz/peg5_frame', PoseStamped, peg5_frame_clbk)


if __name__ == "__main__":

    rospy.init_node('etasl_peg_publisher')

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            
            listener()             
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass    