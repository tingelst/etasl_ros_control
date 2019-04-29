#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import numpy as np
from pyquaternion import Quaternion as quat
import rospy
from std_msgs.msg import Bool, Float64, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

def pub_pose(): 

    rviz_block_frame = rospy.Publisher(
        '/rviz/block_frame', PoseStamped, queue_size=3)
    rviz_peg1_frame = rospy.Publisher(
        '/rviz/peg1_frame', PoseStamped, queue_size=3)
    rviz_peg2_frame = rospy.Publisher(
        '/rviz/peg2_frame', PoseStamped, queue_size=3)
    rviz_peg3_frame = rospy.Publisher(
        '/rviz/peg3_frame', PoseStamped, queue_size=3)
    rviz_peg4_frame = rospy.Publisher(
        '/rviz/peg4_frame', PoseStamped, queue_size=3)
    rviz_peg5_frame = rospy.Publisher(
        '/rviz/peg5_frame', PoseStamped, queue_size=3) 

    rand_quat = quat(axis=[0,0,1], angle=np.pi/2)*quat(axis=[1,0,0], angle=np.random.uniform(0, 2*np.pi))
    block_frame=Pose(
        position=Point(0.0, -0.5, 0.02), 
        orientation=Quaternion(rand_quat.elements[0], rand_quat.elements[1], rand_quat.elements[2], rand_quat.elements[3])
        #orientation=Quaternion(0.5, -0.5, -0.5, 0.5)
        #orientation=Quaternion(0.654021, 0.652994, 0.270263, 0.269839)
    )
    peg1_frame=Pose(
        position=Point(np.random.uniform(-0.22,0.22), np.random.uniform(-0.85,-0.25), 0.02),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
    )
    peg2_frame=Pose(
        position=Point(np.random.uniform(-0.22,0.22), np.random.uniform(-0.85,-0.25), 0.02), 
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
    )
    peg3_frame=Pose(
        position=Point(np.random.uniform(-0.22,0.22), np.random.uniform(-0.85,-0.25), 0.02), 
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
    )      
    peg4_frame=Pose(
        position=Point(np.random.uniform(-0.22,0.22), np.random.uniform(-0.85,-0.25), 0.02), 
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
    )   
    peg5_frame=Pose(
        position=Point(np.random.uniform(-0.22,0.22), np.random.uniform(-0.85,-0.25), 0.02), 
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
    )

    block_frame_s=PoseStamped(
        header=Header(frame_id='base_link', stamp=rospy.Time.now()),
        pose=block_frame
    )
    peg1_frame_s=PoseStamped(
        header=Header(frame_id='base_link', stamp=rospy.Time.now()),
        pose=peg1_frame
    )
    peg2_frame_s=PoseStamped(
        header=Header(frame_id='base_link', stamp=rospy.Time.now()),
        pose=peg2_frame
    )
    peg3_frame_s=PoseStamped(
        header=Header(frame_id='base_link', stamp=rospy.Time.now()),
        pose=peg3_frame
    )      
    peg4_frame_s=PoseStamped(
        header=Header(frame_id='base_link', stamp=rospy.Time.now()),
        pose=peg4_frame
    )   
    peg5_frame_s=PoseStamped(
        header=Header(frame_id='base_link', stamp=rospy.Time.now()),
        pose=peg5_frame
    )    

    rate = rospy.Rate(10)
    i = 0
    try:
        while not rospy.is_shutdown():

            rviz_block_frame.publish(block_frame_s)
            rviz_peg1_frame.publish(peg1_frame_s)
            rviz_peg2_frame.publish(peg2_frame_s)
            rviz_peg3_frame.publish(peg3_frame_s)
            rviz_peg4_frame.publish(peg4_frame_s)
            rviz_peg5_frame.publish(peg5_frame_s)

            if (i > 10):    
                rospy.set_param('picture_taken', 1)
            i += 1
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":

    rospy.init_node('zivid_publisher')
    while not rospy.is_shutdown():
        if (rospy.get_param('take_picture') == 1):
            pub_pose()
            break