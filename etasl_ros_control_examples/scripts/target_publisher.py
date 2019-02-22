#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import time
import numpy as np

import rospy
from std_msgs.msg import Float64, Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3

from copy import deepcopy

pub_tgt_x = rospy.Publisher(
    '/etasl_controller/tgt_x', Float64, queue_size=3)
pub_tgt_y = rospy.Publisher(
    '/etasl_controller/tgt_y', Float64, queue_size=3)
pub_tgt_z = rospy.Publisher(
    '/etasl_controller/tgt_z', Float64, queue_size=3)
pub_marker = rospy.Publisher(
    'visualization_marker', Marker, queue_size=100)

def show_target(marker_publisher, points, color=[0.0, 1.0, 0.0, 0.8], id_=0):
    marker = Marker(
        type=Marker.LINE_STRIP,
        id=id_,
        action=Marker.ADD,
        lifetime=rospy.Duration(1.5),
        points=points,
        scale=Vector3(0.01, 0.0, 0.0),
        header=Header(frame_id='base_link', stamp=rospy.Time.now()),
        color=ColorRGBA(*color),
    )
    marker_publisher.publish(marker)


laser_points = []


def callback_laser(msg):
    global laser_points
    laser_points.append(msg)
    if len(laser_points) > 500:
        laser_points = laser_points[1:]


if __name__ == "__main__":

    rospy.init_node('target_publisher')
    sub_laser = rospy.Subscriber(
        "/etasl_controller/laser", Point, callback_laser)

    rate = rospy.Rate(30)

    points = []

    while not rospy.is_shutdown():
        f1 = 1.0
        f2 = 2.5
        t = time.time()
        tgt_x = np.sin(f1*t)*0.15 + 0.4
        tgt_y = np.sin(f2*t)*0.1 + 0.4
        tgt_z = 0.0
        pub_tgt_x.publish(tgt_x)
        pub_tgt_y.publish(tgt_y)
        pub_tgt_z.publish(tgt_z)
        points.append(Point(tgt_x, tgt_y, tgt_z))

        if len(points) > 250:
            points = points[1:]

        show_target(pub_marker, points, id_=0)
        show_target(pub_marker, deepcopy(laser_points),
                    color=(1.0, 0.0, 0.0, 0.8), id_=1)

        rate.sleep()
