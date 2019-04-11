#!/usr/bin/env python
"""Simple example using services to switch etasl controllers and SMACH for state machine handling."""
import rospy
import numpy as np
import smach
import smach_ros
import os
from zivid_publisher import pub_pose
from etasl_ros_control_msgs.srv import *

from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchControllerResponse
from controller_manager_msgs.srv import SwitchControllerRequest
from std_msgs.msg import *
from geometry_msgs.msg import Pose

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

switching_time = []

def sane_controller_switching(desired_controller, timeout=None):
    """Function to switch hw controller when there can be resource
    conflicts. Potentially slow, but if we don't know what we're
    switching from, it might be necessary.
    """
    rospy.wait_for_service("/controller_manager/list_controllers",
                           timeout=timeout)
    rospy.wait_for_service("/controller_manager/switch_controller",
                           timeout=timeout)
    ls = rospy.ServiceProxy("/controller_manager/list_controllers",
                            ListControllers)
    sw = rospy.ServiceProxy("/controller_manager/switch_controller",
                            SwitchController)
    # Find controllers that control the same resources
    resources = ["joint_a1", "joint_a2", "joint_a3",
                 "joint_a4", "joint_a5", "joint_a6"]
    hw_controllers = ls()
    stop_cntrllrs = []
    for cntrllr in hw_controllers.controller:
        if cntrllr.state == "running":
            if cntrllr.name == desired_controller:
                return SwitchControllerResponse(ok=True)
            for resource in resources:
                try:
                    for claimed_obj in cntrllr.claimed_resources:
                        if resource in claimed_obj.resources:
                            if cntrllr.name not in stop_cntrllrs:
                                stop_cntrllrs.append(cntrllr.name)
                except AttributeError:
                    # Happens with ROS < Kinetic Kame
                    if resource in cntrllr.resources:
                        if cntrllr.name != desired_controller:
                            if cntrllr.name not in stop_cntrllrs:
                                stop_cntrllrs.append(cntrllr.name)
    return sw([str(desired_controller)], stop_cntrllrs,
              SwitchControllerRequest.STRICT)

def activation_command(command):
    rospy.wait_for_service("/etasl_controller_1/activate_cmd", timeout=None)
    sp = rospy.ServiceProxy("/etasl_controller_1/activate_cmd",activate_cmd_service)
    return sp(str(command))

# def pegNdx(ndx):        
#     pegNdx_pub = rospy.Publisher('/etasl_controller_' + str(ndx) + '/pegNdx', Float64, queue_size=3)

#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown(): ##### TODO: wait until status says action complete         
#         pegNdx_pub.publish(ndx)
#         rate.sleep()

finished = False
def gripper_status(status):
    global finished
    if (status.gOBJ == 0):
        finished = False
    else:
        finished = True


def activate_gripper(is_open):
    global finished
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

class Picture(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["picture_taken"])
        self.counter = 1

    def execute(self, userdata):
        rospy.loginfo("Taking a picture")
        rospy.sleep(3.0)
        
        picture = rospy.set_param('take_picture', 1)

        while not rospy.is_shutdown():
            if rospy.get_param('picture_taken') == 1:
                break
        self.counter += 1
        return "picture_taken"

class PickUpLineUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["pickUpLineUp_complete"])
        self.counter = 1

    def execute(self, userdata):
        rospy.loginfo("Lining up for pickup of peg " + str(self.counter))

        rospy.set_param('pegNdx',self.counter)
        i_time = rospy.get_time()
        sane_controller_switching("etasl_controller_1") 
        global switching_time
        switching_time.append(rospy.get_time()-i_time)
        # activation_command("-global.pickup_lineup_1")

        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_1/e_event', String)  
        self.counter += 1
        return "pickUpLineUp_complete"


class PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["pickUp_complete"])

    def execute(self, userdata):
        rospy.loginfo("Approaching peg")
        i_time = rospy.get_time()
        sane_controller_switching("etasl_controller_2") 
        global switching_time
        switching_time.append(rospy.get_time()-i_time)

        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_2/e_event', String)
        return "pickUp_complete"

class Gripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["gripper_closed","gripper_opened"])
        self.is_open = True

    def execute(self, userdata):
        if self.is_open:
            rospy.loginfo("Closing gripper")
            #activate_gripper(self.is_open)
            self.is_open = False
            return "gripper_closed"
        else:
            rospy.loginfo("Opening gripper")
            #activate_gripper(self.is_open)
            self.is_open = True
            return "gripper_opened"

class InsertionLineUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["insertionLineUp_complete","retraction_complete","all_pegs_inserted"])
        self.counter = 1
        self.insert = True

    def execute(self, userdata):
        rospy.loginfo("Lining up for insertion of peg " + str(self.counter))
        i_time = rospy.get_time()
        sane_controller_switching("etasl_controller_3") 
        global switching_time
        switching_time.append(rospy.get_time()-i_time)

        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_3/e_event', String)
        if self.insert:
            self.counter += 1
            self.insert = False
            return "insertionLineUp_complete"
        elif self.counter == 6:
            return "all_pegs_inserted"
        else:
            rospy.sleep(2.0)
            self.insert = True
            return "retraction_complete"

class Insertion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["insertion_complete"])
        self.counter = 1

    def execute(self, userdata):
        rospy.loginfo("Inserting")
        i_time = rospy.get_time()
        sane_controller_switching("etasl_controller_4") 
        global switching_time
        switching_time.append(rospy.get_time()-i_time)

        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_4/e_event', String)
        self.counter += 1
        return "insertion_complete"
        
class Home(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["home_reached"])

    def execute(self, userdata):
        rospy.loginfo("Home")
        i_time = rospy.get_time()
        sane_controller_switching("etasl_controller_home") 
        global switching_time
        switching_time.append(rospy.get_time()-i_time)

        rospy.loginfo(np.average(switching_time))

        rospy.sleep(20.0)
        return "home_reached"        


if __name__ == "__main__":
    rospy.init_node("smach_peg")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["fully_complete"])
    with sm:
        smach.StateMachine.add("Picture", Picture(),
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
        rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, gripper_status)
        rospy.spin()
    except rospy.ROSInterruptException:
        sis.stop()
        
