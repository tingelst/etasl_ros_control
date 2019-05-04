#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

"""Simple example using services to switch etasl controllers and SMACH for state machine handling."""
import rospy
import numpy as np
import matplotlib.pyplot as plt
import smach
import smach_ros
import os
from zivid_publisher import pub_pose
from etasl_ros_control_msgs.srv import *

from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchControllerResponse
from controller_manager_msgs.srv import SwitchControllerRequest
from std_msgs.msg import *
from geometry_msgs.msg import Pose, Wrench

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

switching_time = []

def activate_cmd_switch(command):
    global switching_time
    i_time = rospy.get_time()
    stop()
    rospy.wait_for_service('/etasl_controller_cmd/activate_cmd')
    activate_cmd = rospy.ServiceProxy('/etasl_controller_cmd/activate_cmd', Command)
    resp = activate_cmd(command)
    start()
    
    switching_time.append(rospy.get_time()-i_time)
    return resp.ok

def start():
    rospy.wait_for_service("/controller_manager/switch_controller")
    start = rospy.ServiceProxy("/controller_manager/switch_controller",SwitchController)
    start(["etasl_controller_cmd"],[],SwitchControllerRequest.STRICT)

def stop():
    rospy.wait_for_service("/controller_manager/switch_controller")
    start = rospy.ServiceProxy("/controller_manager/switch_controller",SwitchController)
    start([],["etasl_controller_cmd"],SwitchControllerRequest.STRICT)

def plot_switch_time(switching_time):
    plt.figure()
    plt.plot(switching_time,'*k')
    plt.plot([0,30],[np.average(switching_time),np.average(switching_time)],'r')
    plt.show()

finished = False
def gripper_status(status):
    global finished
    if (status.gOBJ == 0):
        finished = False
    else:
        finished = True


def activate_gripper(is_open):
    global finished
    finished = False
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=3)

    command = outputMsg.Robotiq2FGripper_robot_output()
    """ Init command """
    command.rACT = 1
    command.rGTO = 1
    command.rATR = 0
    command.rSP = 255 # Max speed
    command.rFR = 255 # Max force    

    if is_open:
        command.rPR = 255 #I.e. closing gripper
    elif not is_open:
        command.rPR = 0 #I.e. opening gripper
    while not finished:        
        pub.publish(command)
        rate.sleep()

class TakePicture(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["picture_taken"])
        self.counter = 1

    def execute(self, userdata):
        rospy.sleep(5.0)
        
        rospy.wait_for_service('take_picture')
        try:
            take_picture = rospy.ServiceProxy('take_picture', Picture)
            resp = take_picture(True)
            if resp.picture_taken:
                self.counter += 1
                return "picture_taken"
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)

class PickUpLineUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["pickUpLineUp_complete"])
        self.counter = 1

    def execute(self, userdata):
        if (self.counter == 1):
            start()
        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_cmd/e_event', String)  
        self.counter += 1
        return "pickUpLineUp_complete"


class PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["pickUp_complete"])
        self.counter = 1

    def execute(self, userdata):
        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_cmd/e_event', String)
        self.counter += 1
        return "pickUp_complete"

class Gripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["gripper_closed","gripper_opened"])
        self.counter = 1
        self.is_open = True

    def execute(self, userdata):
        if rospy.get_param("connection"):
            activate_gripper(self.is_open)
        else:
            rospy.sleep(2.0)
        if self.is_open:
            activate_cmd_switch("+global.insertion_lineup_"+str(self.counter))
            self.is_open = False
            return "gripper_closed"
        else:
            activate_cmd_switch("+global.retract_"+str(self.counter))
            self.counter += 1
            self.is_open = True
            return "gripper_opened"

class InsertionLineUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["insertionLineUp_complete","retraction_complete","all_pegs_inserted"])
        self.counter = 1
        self.insert = True

    def execute(self, userdata):
        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_cmd/e_event', String)
        if self.insert:
            self.insert = False
            return "insertionLineUp_complete"
        elif self.counter == 5:
            activate_cmd_switch("+global.home")
            return "all_pegs_inserted"
        else:
            self.counter += 1
            self.insert = True
            return "retraction_complete"

class Insertion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["insertion_complete"])
        self.counter = 1

    def execute(self, userdata):
        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_cmd/e_event', String)
        self.counter += 1
        return "insertion_complete"

        
class Home(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["home_reached"])

    def execute(self, userdata):
        global switching_time
        plot_switch_time(switching_time)

        rospy.sleep(5.0)
        return "home_reached"        


if __name__ == "__main__":
    rospy.init_node("smach_peg")
    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, gripper_status)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["fully_complete"])
    with sm:
        smach.StateMachine.add("TakePicture", TakePicture(),
                               transitions={"picture_taken": "PickUpLineUp"})
        smach.StateMachine.add("PickUpLineUp", PickUpLineUp(),
                               transitions={"pickUpLineUp_complete": "PickUp"})
        smach.StateMachine.add("PickUp", PickUp(),
                               transitions={"pickUp_complete": "Gripper"}) 
        smach.StateMachine.add("Gripper", Gripper(),
                               transitions={"gripper_closed": "InsertionLineUp",
                                            "gripper_opened": "InsertionLineUp"}) 
        smach.StateMachine.add("InsertionLineUp", InsertionLineUp(),
                               transitions={"insertionLineUp_complete": "Insertion",
                                            "retraction_complete": "PickUpLineUp",
                                            "all_pegs_inserted": "Home"})     
        smach.StateMachine.add("Insertion", Insertion(),
                               transitions={"insertion_complete": "Gripper"})    
        smach.StateMachine.add("Home", Home(),
                               transitions={"home_reached": "fully_complete"})                                                 
    sis = smach_ros.IntrospectionServer('peg_in_hole', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        sis.stop()
        
