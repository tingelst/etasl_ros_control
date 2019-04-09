#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import numpy as np

import rospy
from std_msgs.msg import Float64, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

ndx = 4

etasl_block_frame = rospy.Publisher(
    '/etasl_controller_' + str(ndx) + '/block_frame', Pose, queue_size=3)

def block_frame_clbk(data):
    block_frame = data.pose
    etasl_block_frame.publish(block_frame)

def listener():
    rospy.Subscriber('/rviz/block_frame', PoseStamped, block_frame_clbk)

if __name__ == "__main__":

    rospy.init_node('etasl_publisher_' + str(ndx))

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            
            listener()             
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass    