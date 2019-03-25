#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import time
import numpy as np

import rospy
from std_msgs.msg import Float64

pub_tgt_x = rospy.Publisher(
    '/etasl_controller/tgt_x', Float64, queue_size=3)
pub_tgt_y = rospy.Publisher(
    '/etasl_controller/tgt_y', Float64, queue_size=3)
pub_tgt_z = rospy.Publisher(
    '/etasl_controller/tgt_z', Float64, queue_size=3)

if __name__ == "__main__":

    rospy.init_node('target_publisher')

    rate = rospy.Rate(30)

    try:
        while not rospy.is_shutdown():

            f1 = 20
            f2 = 50
            t = time.time()
            tgt_x = np.sin(f1*t)*0.0015
            tgt_y = np.sin(f2*t)*0.001
            tgt_z = 0.0
            pub_tgt_x.publish(tgt_x)
            pub_tgt_y.publish(tgt_y)
            pub_tgt_z.publish(tgt_z)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
