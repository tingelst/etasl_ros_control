#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from std_msgs.msg import Float64, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

peg_etasl_1 = rospy.Publisher(
    '/etasl_controller_1/peg_frame', Pose, queue_size=3)
peg_etasl_2 = rospy.Publisher(
    '/etasl_controller_2/peg_frame', Pose, queue_size=3)
block_etasl_3 = rospy.Publisher(
    '/etasl_controller_3/block_frame', Pose, queue_size=3)
block_etasl_4 = rospy.Publisher(
    '/etasl_controller_4/block_frame', Pose, queue_size=3)
block_etasl_lissajous = rospy.Publisher(
    '/etasl_controller_lissajous/block_frame', Pose, queue_size=3)
rviz_block = rospy.Publisher(
    '/rviz/block', Marker, queue_size=100)    
rviz_peg1 = rospy.Publisher(
    '/rviz/peg1', Marker, queue_size=100)
rviz_peg2 = rospy.Publisher(
    '/rviz/peg2', Marker, queue_size=100)
rviz_peg3 = rospy.Publisher(
    '/rviz/peg3', Marker, queue_size=100)
rviz_peg4 = rospy.Publisher(
    '/rviz/peg4', Marker, queue_size=100)
rviz_peg5 = rospy.Publisher(
    '/rviz/peg5', Marker, queue_size=100) 

def peg1_frame_clbk(data):
    peg1_frame = data.pose 

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
    rviz_peg1.publish(peg1_marker)

    if (rospy.get_param('pegNdx') == 1):
        peg_etasl_1.publish(peg1_frame)
        peg_etasl_2.publish(peg1_frame)

def peg2_frame_clbk(data):
    peg2_frame = data.pose 

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
    rviz_peg2.publish(peg2_marker)

    if (rospy.get_param('pegNdx') == 2):
        peg_etasl_1.publish(peg2_frame)
        peg_etasl_2.publish(peg2_frame)

def peg3_frame_clbk(data):
    peg3_frame = data.pose 

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
    rviz_peg3.publish(peg3_marker)

    if (rospy.get_param('pegNdx') == 3):
        peg_etasl_1.publish(peg3_frame)
        peg_etasl_2.publish(peg3_frame)

def peg4_frame_clbk(data):
    peg4_frame = data.pose 

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
    rviz_peg4.publish(peg4_marker)

    if (rospy.get_param('pegNdx') == 4):
        peg_etasl_1.publish(peg4_frame)
        peg_etasl_2.publish(peg4_frame)

def peg5_frame_clbk(data):
    peg5_frame = data.pose 

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
    rviz_peg5.publish(peg5_marker)

    if (rospy.get_param('pegNdx') == 5):
        peg_etasl_1.publish(peg5_frame)
        peg_etasl_2.publish(peg5_frame)

def block_frame_clbk(data):
    block_pos = data.pose.position
    block_ori = data.pose.orientation

    rot = R.from_quat([block_ori.x, block_ori.y, block_ori.z, block_ori.w])
    trans = np.dot(rot.as_dcm(),([[0.025 + 0.05*(rospy.get_param('pegNdx')-1)], [0], [-0.025]]))

    block_frame=Pose(
        position=Point(block_pos.x + trans[0,0], block_pos.y + trans[1,0], block_pos.z + trans[2,0]),
        orientation=Quaternion(block_ori.x, block_ori.y, block_ori.z, block_ori.w)
    )

    block_marker=Marker(
        header=Header(frame_id='base_link', stamp=rospy.Time.now()),
        id=0, 
        type=Marker.MESH_RESOURCE,
        action=Marker.ADD,
        pose=data.pose,
        scale=Vector3(1.0, 1.0, 1.0),
        color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
        lifetime=rospy.Duration(),
        mesh_resource="package://etasl_ros_control_examples/mesh/block.stl",
        frame_locked=1
    )

    block_etasl_3.publish(block_frame)
    block_etasl_4.publish(block_frame)
    block_etasl_lissajous.publish(block_frame)
    rviz_block.publish(block_marker)


def listener():
    rospy.init_node('etasl_pose_publisher')

    rospy.Subscriber('/rviz/peg1_frame', PoseStamped, peg1_frame_clbk)
    rospy.Subscriber('/rviz/peg2_frame', PoseStamped, peg2_frame_clbk)
    rospy.Subscriber('/rviz/peg3_frame', PoseStamped, peg3_frame_clbk)
    rospy.Subscriber('/rviz/peg4_frame', PoseStamped, peg4_frame_clbk)
    rospy.Subscriber('/rviz/peg5_frame', PoseStamped, peg5_frame_clbk)
    rospy.Subscriber('/rviz/block_frame', PoseStamped, block_frame_clbk)

    rospy.spin() 


if __name__ == "__main__":
    listener()