#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import rospy
import numpy as np
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Float64

def wrench_publisher():
    rospy.init_node('wrench_publisher') 

    wrench_pub = rospy.Publisher('/etasl_controller_cmd/netft_data', Wrench, queue_size=100)
    rate = rospy.Rate(250)
    try:
        while not rospy.is_shutdown():
            f = 0.2
            t = rospy.get_time()
            f_z = np.abs(np.sin(f*t)*100)

            wrench = Wrench(
                force=Vector3(0, 0, f_z),
                torque=Vector3(0, 0, 0)
            )
            wrench_pub.publish(wrench)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass  

if __name__ == "__main__":
    wrench_publisher()

    