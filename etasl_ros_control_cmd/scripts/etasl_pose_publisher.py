#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import rospy
from std_msgs.msg import Float64, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from etasl_ros_control_msgs.srv import *

pos_peg1 = 0
pos_peg2 = 0
pos_peg3 = 0
pos_peg4 = 0
pos_peg5 = 0

peg_etasl_1 = rospy.Publisher(
    '/etasl_controller_cmd/peg1_frame', Pose, queue_size=3)
peg_etasl_2 = rospy.Publisher(
    '/etasl_controller_cmd/peg2_frame', Pose, queue_size=3) 
peg_etasl_3 = rospy.Publisher(
    '/etasl_controller_cmd/peg3_frame', Pose, queue_size=3)
peg_etasl_4 = rospy.Publisher(
    '/etasl_controller_cmd/peg4_frame', Pose, queue_size=3)
peg_etasl_5 = rospy.Publisher(
    '/etasl_controller_cmd/peg5_frame', Pose, queue_size=3)   
block_etasl = rospy.Publisher(
    '/etasl_controller_cmd/block_frame', Pose, queue_size=3)
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

peg1_marker=Marker(
    id=1, 
    type=Marker.MESH_RESOURCE,
    action=Marker.ADD,
    scale=Vector3(1.0, 1.0, 1.0),
    color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
    lifetime=rospy.Duration(),
    mesh_resource="package://etasl_ros_control_examples/mesh/peg.stl",
    frame_locked=1
)
peg2_marker=Marker(
    id=2, 
    type=Marker.MESH_RESOURCE,
    action=Marker.ADD,
    scale=Vector3(1.0, 1.0, 1.0),
    color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
    lifetime=rospy.Duration(),
    mesh_resource="package://etasl_ros_control_examples/mesh/peg.stl",
    frame_locked=1
) 
peg3_marker=Marker(
    id=3, 
    type=Marker.MESH_RESOURCE,
    action=Marker.ADD,
    scale=Vector3(1.0, 1.0, 1.0),
    color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
    lifetime=rospy.Duration(),
    mesh_resource="package://etasl_ros_control_examples/mesh/peg.stl",
    frame_locked=1
)  
peg4_marker=Marker(
    id=4, 
    type=Marker.MESH_RESOURCE,
    action=Marker.ADD,
    scale=Vector3(1.0, 1.0, 1.0),
    color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
    lifetime=rospy.Duration(),
    mesh_resource="package://etasl_ros_control_examples/mesh/peg.stl",
    frame_locked=1
)  
peg5_marker=Marker(
    id=5, 
    type=Marker.MESH_RESOURCE,
    action=Marker.ADD,
    scale=Vector3(1.0, 1.0, 1.0),
    color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
    lifetime=rospy.Duration(),
    mesh_resource="package://etasl_ros_control_examples/mesh/peg.stl",
    frame_locked=1
)  
block_marker=Marker(
    id=6, 
    type=Marker.MESH_RESOURCE,
    action=Marker.ADD,
    scale=Vector3(1.0, 1.0, 1.0),
    color=ColorRGBA(0.263, 0.275, 0.294, 1.0),
    lifetime=rospy.Duration(),
    mesh_resource="package://etasl_ros_control_examples/mesh/block.stl",
    frame_locked=1
)

def peg_pos(req):
    global pos_peg1
    global pos_peg2
    global pos_peg3
    global pos_peg4
    global pos_peg5
    pos_peg1 = req.peg1_pos
    pos_peg2 = req.peg2_pos
    pos_peg3 = req.peg3_pos
    pos_peg4 = req.peg4_pos
    pos_peg5 = req.peg5_pos
    return PegPosResponse(True)

def peg1_frame_clbk(data):
    global pos_peg1
    if pos_peg1 == 0:
        peg_etasl_1.publish(data.pose)
        peg1_marker.header = data.header
        peg1_marker.pose = data.pose        
        rviz_peg1.publish(peg1_marker)

def peg2_frame_clbk(data):
    global pos_peg2
    if pos_peg2 == 0:
        peg_etasl_2.publish(data.pose)
        peg2_marker.header = data.header
        peg2_marker.pose = data.pose        
        rviz_peg2.publish(peg2_marker)

def peg3_frame_clbk(data):
    global pos_peg3
    if pos_peg3 == 0:
        peg_etasl_3.publish(data.pose)
        peg3_marker.header = data.header
        peg3_marker.pose = data.pose        
        rviz_peg3.publish(peg3_marker)

def peg4_frame_clbk(data):
    global pos_peg4
    if pos_peg4 == 0:
        peg_etasl_4.publish(data.pose)
        peg4_marker.header = data.header
        peg4_marker.pose = data.pose        
        rviz_peg4.publish(peg4_marker)

def peg5_frame_clbk(data):
    global pos_peg5
    if pos_peg5 == 0:
        peg_etasl_5.publish(data.pose)
        peg5_marker.header = data.header
        peg5_marker.pose = data.pose        
        rviz_peg5.publish(peg5_marker)

def peg_hole_1_clbk(data):
    global pos_peg1
    if pos_peg1 == 2:
        peg1_marker.header = Header(frame_id='base_link',stamp=rospy.Time.now())
        peg1_marker.pose = data
        rviz_peg1.publish(peg1_marker)

def peg_hole_2_clbk(data):
    global pos_peg2
    if pos_peg2 == 2:
        peg2_marker.header = Header(frame_id='base_link',stamp=rospy.Time.now())
        peg2_marker.pose = data
        rviz_peg2.publish(peg2_marker)

def peg_hole_3_clbk(data):
    global pos_peg3
    if pos_peg3 == 2:
        peg3_marker.header = Header(frame_id='base_link',stamp=rospy.Time.now())
        peg3_marker.pose = data
        rviz_peg3.publish(peg3_marker)

def peg_hole_4_clbk(data):
    global pos_peg4
    if pos_peg4 == 2:
        peg4_marker.header = Header(frame_id='base_link',stamp=rospy.Time.now())
        peg4_marker.pose = data
        rviz_peg4.publish(peg4_marker)

def peg_hole_5_clbk(data):
    global pos_peg5
    if pos_peg5 == 2:
        peg5_marker.header = Header(frame_id='base_link',stamp=rospy.Time.now())
        peg5_marker.pose = data
        rviz_peg5.publish(peg5_marker)       

def block_frame_clbk(data):
    block_etasl.publish(data.pose)
    block_marker.header = data.header
    block_marker.pose = data.pose
    rviz_block.publish(block_marker)

def peg_in_gripper_clbk(data):
    global pos_peg1
    global pos_peg2
    global pos_peg3
    global pos_peg4
    global pos_peg5
    if pos_peg1 == 1:
        peg1_marker.header = Header(frame_id='base_link',stamp=rospy.Time.now())
        peg1_marker.pose = data
        rviz_peg1.publish(peg1_marker)
    elif pos_peg2 == 1:
        peg2_marker.header = Header(frame_id='base_link',stamp=rospy.Time.now())
        peg2_marker.pose = data
        rviz_peg2.publish(peg2_marker)
    elif pos_peg3 == 1:
        peg3_marker.header = Header(frame_id='base_link',stamp=rospy.Time.now())
        peg3_marker.pose = data
        rviz_peg3.publish(peg3_marker)
    elif pos_peg4 == 1:
        peg4_marker.header = Header(frame_id='base_link',stamp=rospy.Time.now())
        peg4_marker.pose = data
        rviz_peg4.publish(peg4_marker)
    elif pos_peg5 == 1:
        peg5_marker.header = Header(frame_id='base_link',stamp=rospy.Time.now())
        peg5_marker.pose = data
        rviz_peg5.publish(peg5_marker)

def listener():
    rospy.init_node('etasl_pose_publisher') 

    rospy.Subscriber('/rviz/peg1_frame', PoseStamped, peg1_frame_clbk)
    rospy.Subscriber('/rviz/peg2_frame', PoseStamped, peg2_frame_clbk)
    rospy.Subscriber('/rviz/peg3_frame', PoseStamped, peg3_frame_clbk)
    rospy.Subscriber('/rviz/peg4_frame', PoseStamped, peg4_frame_clbk)
    rospy.Subscriber('/rviz/peg5_frame', PoseStamped, peg5_frame_clbk)
    rospy.Subscriber('/rviz/block_frame', PoseStamped, block_frame_clbk)
    
    rospy.Subscriber('etasl_controller_cmd/ee_frame', Pose, peg_in_gripper_clbk)
    rospy.Subscriber('etasl_controller_cmd/peg_hole_1', Pose, peg_hole_1_clbk)
    rospy.Subscriber('etasl_controller_cmd/peg_hole_2', Pose, peg_hole_2_clbk)
    rospy.Subscriber('etasl_controller_cmd/peg_hole_3', Pose, peg_hole_3_clbk)
    rospy.Subscriber('etasl_controller_cmd/peg_hole_4', Pose, peg_hole_4_clbk)
    rospy.Subscriber('etasl_controller_cmd/peg_hole_5', Pose, peg_hole_5_clbk)

    rospy.Service('peg_pos', PegPos, peg_pos)

    rospy.spin() 

if __name__ == "__main__":
    listener()