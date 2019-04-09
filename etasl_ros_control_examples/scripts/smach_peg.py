#!/usr/bin/env python
"""Simple example using services to switch etasl controllers and SMACH for state machine handling."""
import rospy
import smach
import smach_ros
from etasl_ros_control_msgs.srv import activate_cmd_service

from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchControllerResponse
from controller_manager_msgs.srv import SwitchControllerRequest
from std_msgs.msg import *

# import roslib; roslib.load_manifest('robotiq_2f_gripper_control') 
# roslib.load_manifest('robotiq_modbus_tcp')
# import robotiq_2f_gripper_control.baseRobotiq2FGripper
# import robotiq_modbus_tcp.comModbusTcp
# import os, sys
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

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

# def activateGripper(closing):
#     address = rospy.get_param('address')
#     gripper = robotiq_2f_gripper_control.baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
#     gripper.client = robotiq_modbus_tcp.comModbusTcp.communication()
#     gripper.client.connectToDevice(address)
#     gripper.client.wait_for_server()

#     if closing:
#         Robotiq.close(gripper.client(), force=255)
#     if not closing:
#         Robotiq.open(gripper.client())

class PickUpLineUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["pickUpLineUp_complete"])
        self.counter = 1

    def execute(self, userdata):
        rospy.loginfo("Lining up for pickup")
        rospy.sleep(1.0)
        sane_controller_switching("etasl_controller_1")        
        # activation_command("-global.pickup_lineup_1")
        #rospy.sleep(30.0)

        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_1/e_event', String)  
        self.counter += 1
        return "pickUpLineUp_complete"


class PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["pickUp_complete"])

    def execute(self, userdata):
        rospy.loginfo("Approaching peg")
        sane_controller_switching("etasl_controller_2")
        #rospy.sleep(10.0)

        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_2/e_event', String)
        return "pickUp_complete"

class Gripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["gripper_closed","gripper_opened"])
        self.open = True

    def execute(self, userdata):
        rospy.sleep(2.0)
        if self.open:
            rospy.loginfo("Closing gripper")
            #activateGripper(open)
            self.open = False
            return "gripper_closed"
        else:
            rospy.loginfo("Opening gripper")
            #activateGripper(open)
            self.open = True
            return "gripper_opened"

class InsertionLineUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["insertionLineUp_complete","retraction_complete","all_pegs_inserted"])
        self.counter = 1
        self.insert = True

    def execute(self, userdata):
        rospy.loginfo("Lining up for insertion")
        sane_controller_switching("etasl_controller_3")
        #rospy.sleep(10.0)

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
        sane_controller_switching("etasl_controller_4")
        #rospy.sleep(10.0)

        #Wait until exit event is published
        rospy.wait_for_message('/etasl_controller_4/e_event', String)
        self.counter += 1
        return "insertion_complete"
        
class Home(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["home_reached"])

    def execute(self, userdata):
        rospy.loginfo("Home")
        sane_controller_switching("etasl_controller_home")
        rospy.sleep(20.0)
        return "home_reached"        


if __name__ == "__main__":
    rospy.init_node("smach_peg")
    #listener()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["fully_complete"])
    with sm:
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
        
