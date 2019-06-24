#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import rospy
from std_msgs.msg import Float64

def speed_publisher():

    rospy.init_node('speed_publisher')
    speed = rospy.get_param('~gain_multiplier')

    speed1 = rospy.Publisher('/etasl_controller_cmd/speed', Float64, queue_size=3)

    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            
            speed1.publish(speed)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass    

if __name__ == "__main__":
    speed_publisher()