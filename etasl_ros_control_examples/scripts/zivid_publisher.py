#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import numpy as np

import rospy
from std_msgs.msg import Float64, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3


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
block = rospy.Publisher(
    '/rviz/block', Marker, queue_size=100)    
peg1 = rospy.Publisher(
    '/rviz/peg1', Marker, queue_size=100)
peg2 = rospy.Publisher(
    '/rviz/peg2', Marker, queue_size=100)
peg3 = rospy.Publisher(
    '/rviz/peg3', Marker, queue_size=100)
peg4 = rospy.Publisher(
    '/rviz/peg4', Marker, queue_size=100)
peg5 = rospy.Publisher(
    '/rviz/peg5', Marker, queue_size=100)    

if __name__ == "__main__":

    rospy.init_node('zivid_publisher')

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            block_frame=Pose(
                position=Point(0.0, 0.5-1.0, 0.02), 
                #orientation=Quaternion(np.pi/2.0, 0.0, 0.0, np.pi/2.0)
                orientation=Quaternion(0.5, -0.5, -0.5, 0.5)
                #orientation=Quaternion(0.654021, 0.652994, 0.270263, 0.269839)
            )
            peg1_frame=Pose(
                position=Point(0.3, 0.4-1.0, 0.02), 
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
            peg2_frame=Pose(
                position=Point(0.5, 0.4-1.0, 0.02),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )
            peg3_frame=Pose(
                position=Point(0.35, 0.5-1.0, 0.02), 
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )      
            peg4_frame=Pose(
                position=Point(0.45, 0.57-1.0, 0.02), 
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
            )   
            peg5_frame=Pose(
                position=Point(0.2, 0.6-1.0, 0.02), 
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

            rviz_block_frame.publish(block_frame_s)
            rviz_peg1_frame.publish(peg1_frame_s)
            rviz_peg2_frame.publish(peg2_frame_s)
            rviz_peg3_frame.publish(peg3_frame_s)
            rviz_peg4_frame.publish(peg4_frame_s)
            rviz_peg5_frame.publish(peg5_frame_s)

            block_marker=Marker(
                header=Header(frame_id='base_link', stamp=rospy.Time.now()),
                id=0, 
                type=Marker.MESH_RESOURCE,
                action=Marker.ADD,
                pose=block_frame,
                scale=Vector3(1.0, 1.0, 1.0),
                color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
                lifetime=rospy.Duration(),
                mesh_resource="package://etasl_ros_control_examples/mesh/block.stl",
                frame_locked=1
            )
            peg1_marker=Marker(
                header=Header(frame_id='base_link', stamp=rospy.Time.now()),
                id=0, 
                type=Marker.MESH_RESOURCE,
                action=Marker.ADD,
                pose=peg1_frame,
                scale=Vector3(1.0, 1.0, 1.0),
                color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
                lifetime=rospy.Duration(),
                mesh_resource="package://etasl_ros_control_examples/mesh/peg.stl",
                frame_locked=1
            )
            peg2_marker=Marker(
                header=Header(frame_id='base_link', stamp=rospy.Time.now()),
                id=0, 
                type=Marker.MESH_RESOURCE,
                action=Marker.ADD,
                pose=peg2_frame,
                scale=Vector3(1.0, 1.0, 1.0),
                color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
                lifetime=rospy.Duration(),
                mesh_resource="package://etasl_ros_control_examples/mesh/peg.stl",
                frame_locked=1
            )
            peg3_marker=Marker(
                header=Header(frame_id='base_link', stamp=rospy.Time.now()),
                id=0, 
                type=Marker.MESH_RESOURCE,
                action=Marker.ADD,
                pose=peg3_frame,
                scale=Vector3(1.0, 1.0, 1.0),
                color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
                lifetime=rospy.Duration(),
                mesh_resource="package://etasl_ros_control_examples/mesh/peg.stl",
                frame_locked=1
            )
            peg4_marker=Marker(
                header=Header(frame_id='base_link', stamp=rospy.Time.now()),
                id=0, 
                type=Marker.MESH_RESOURCE,
                action=Marker.ADD,
                pose=peg4_frame,
                scale=Vector3(1.0, 1.0, 1.0),
                color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
                lifetime=rospy.Duration(),
                mesh_resource="package://etasl_ros_control_examples/mesh/peg.stl",
                frame_locked=1
            )
            peg5_marker=Marker(
                header=Header(frame_id='base_link', stamp=rospy.Time.now()),
                id=0, 
                type=Marker.MESH_RESOURCE,
                action=Marker.ADD,
                pose=peg5_frame,
                scale=Vector3(1.0, 1.0, 1.0),
                color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
                lifetime=rospy.Duration(),
                mesh_resource="package://etasl_ros_control_examples/mesh/peg.stl",
                frame_locked=1
            )
            block.publish(block_marker)
            peg1.publish(peg1_marker)
            peg2.publish(peg2_marker)
            peg3.publish(peg3_marker)
            peg4.publish(peg4_marker)
            peg5.publish(peg5_marker)
    

            rate.sleep()
    except rospy.ROSInterruptException:
        pass