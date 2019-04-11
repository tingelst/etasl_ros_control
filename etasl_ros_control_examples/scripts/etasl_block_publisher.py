#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from std_msgs.msg import Float64, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

block_etasl_3 = rospy.Publisher(
    '/etasl_controller_3/block_frame', Pose, queue_size=3)
block_etasl_4 = rospy.Publisher(
    '/etasl_controller_4/block_frame', Pose, queue_size=3)

def block_frame_clbk(data):
    block_pos = data.pose.position
    block_ori = data.pose.orientation

    rot = R.from_quat([block_ori.x, block_ori.y, block_ori.z, block_ori.w])
    trans = np.dot(rot.as_dcm(),([[0.025 + 0.05*(rospy.get_param('pegNdx')-1)], [0], [-0.025]]))

    block_frame=Pose(
        position=Point(block_pos.x + trans[0,0], block_pos.y + trans[1,0], block_pos.z + trans[2,0]),
        orientation=Quaternion(block_ori.x, block_ori.y, block_ori.z, block_ori.w)
    )
    block_etasl_3.publish(block_frame)
    block_etasl_4.publish(block_frame)

def listener():
    rospy.Subscriber('/rviz/block_frame', PoseStamped, block_frame_clbk)

if __name__ == "__main__":

    rospy.init_node('etasl_block_publisher')

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            
            listener()             
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass    