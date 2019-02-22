#!/usr/bin/env python

import time
import numpy as np
import rospy
from std_msgs.msg import Float64

if __name__ == "__main__":

    rospy.init_node('target_publisher')

    pub_tgt_x = rospy.Publisher(
        '/etasl_controller/tgt_x', Float64, queue_size=3)
    pub_tgt_y = rospy.Publisher(
        '/etasl_controller/tgt_y', Float64, queue_size=3)
    pub_tgt_z = rospy.Publisher(
        '/etasl_controller/tgt_z', Float64, queue_size=3)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        try:
            f1 = 1.0
            f2 = 2.5
            t = time.time()
            tgt_x = np.sin(f1*t)*0.15 + 0.7
            tgt_y = np.sin(f2*t)*0.1 + 0.4
            tgt_z = 0.0
            pub_tgt_x.publish(tgt_x)
            pub_tgt_y.publish(tgt_y)
            pub_tgt_z.publish(tgt_z)
            rate.sleep()
        except:
            pass
