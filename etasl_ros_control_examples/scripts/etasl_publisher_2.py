#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import numpy as np

import rospy
from std_msgs.msg import Float64, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

ndx = 2

etasl_peg1_frame = rospy.Publisher(
    '/etasl_controller_' + str(ndx) + '/peg1_frame', Pose, queue_size=3)
etasl_peg2_frame = rospy.Publisher(
    '/etasl_controller_' + str(ndx) + '/peg2_frame', Pose, queue_size=3)
etasl_peg3_frame = rospy.Publisher(
    '/etasl_controller_' + str(ndx) + '/peg3_frame', Pose, queue_size=3)
etasl_peg4_frame = rospy.Publisher(
    '/etasl_controller_' + str(ndx) + '/peg4_frame', Pose, queue_size=3)
etasl_peg5_frame = rospy.Publisher(
    '/etasl_controller_' + str(ndx) + '/peg5_frame', Pose, queue_size=3)

def peg1_frame_clbk(data):
    peg1_frame = data.pose 
    etasl_peg1_frame.publish(peg1_frame)

def peg2_frame_clbk(data):
    peg2_frame = data.pose 
    etasl_peg2_frame.publish(peg2_frame)

def peg3_frame_clbk(data):
    peg3_frame = data.pose 
    etasl_peg3_frame.publish(peg3_frame)

def peg4_frame_clbk(data):
    peg4_frame = data.pose 
    etasl_peg4_frame.publish(peg4_frame)

def peg5_frame_clbk(data):
    peg5_frame = data.pose 
    etasl_peg5_frame.publish(peg5_frame)

def listener():
    rospy.Subscriber('/rviz/peg1_frame', PoseStamped, peg1_frame_clbk)
    rospy.Subscriber('/rviz/peg2_frame', PoseStamped, peg2_frame_clbk)
    rospy.Subscriber('/rviz/peg3_frame', PoseStamped, peg3_frame_clbk)
    rospy.Subscriber('/rviz/peg4_frame', PoseStamped, peg4_frame_clbk)
    rospy.Subscriber('/rviz/peg5_frame', PoseStamped, peg5_frame_clbk)


if __name__ == "__main__":

    rospy.init_node('etasl_publisher_' + str(ndx))

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            
            listener()             
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass    