#!/usr/bin/env python

#  Copyright (c) 2019 Norwegian University of Science and Technology
#  Use of this source code is governed by the LGPL-3.0 license, see LICENSE

import rospy
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

# Service to switch active groups in the eTaSL context
def activate_cmd_switch(command):
    i_time = rospy.get_time()
    
    rospy.wait_for_service('/etasl_controller_cmd/activate_cmd')
    activate_cmd = rospy.ServiceProxy('/etasl_controller_cmd/activate_cmd', Command)
    resp = activate_cmd(command)
    
    rospy.loginfo("Switching time srv: " + str(rospy.get_time()-i_time))
    return resp.ok
# Service to start the controller
def start():
    rospy.wait_for_service("/controller_manager/switch_controller")
    start = rospy.ServiceProxy("/controller_manager/switch_controller",SwitchController)
    start(["etasl_controller_cmd"],[],SwitchControllerRequest.STRICT)
# Service to stop the controller
def stop():
    rospy.wait_for_service("/controller_manager/switch_controller")
    stop = rospy.ServiceProxy("/controller_manager/switch_controller",SwitchController)
    stop([],["etasl_controller_cmd"],SwitchControllerRequest.STRICT)

def peg_posititon(p1,p2,p3,p4,p5):
    rospy.wait_for_service('peg_pos')
    pegs = rospy.ServiceProxy('peg_pos', PegPos)
    resp = pegs(int(p1),int(p2),int(p3),int(p4),int(p5))

finished = False
def gripper_status(status):
    global finished
    finished = False
    if (status.gOBJ == 0):
        finished = False
    elif (status.gOBJ > 0):
        finished = True


def activate_gripper(is_open):
    global finished
    finished = False
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=3)

    command = outputMsg.Robotiq2FGripper_robot_output()
    """ Init command """
    command.rACT = 1 # Activate
    command.rGTO = 1 
    command.rATR = 0
    command.rSP = 255 # Max speed
    command.rFR = 255 # Max force    

    rate = rospy.Rate(250)
    time_i = rospy.get_time()
    if is_open:
        command.rPR = 255 #I.e. closing gripper
    elif not is_open:
        command.rPR = 170 #I.e. opening gripper
    while not finished:        
        pub.publish(command)
        rate.sleep()
        if (rospy.get_time()-time_i > 5):
            return False

class TakePicture(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["picture_taken","retry_picture"])

    def execute(self, userdata):        
        rospy.wait_for_service('take_picture')
        try:
            take_picture = rospy.ServiceProxy('take_picture', Picture)
            resp = take_picture(True)
            if resp.picture_taken:
                return "picture_taken"
            else:
                return "retry_picture"
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)

# Nothing executed in this state. 
# Only a contianer for visualization of eTaSL states in smashviz
class PickUpLineUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["pickUpLineUp_complete"])
        self.counter = 1

    def execute(self, userdata):
        if (self.counter == 1):
            start()
            if rospy.get_param("connection"):
                activate_gripper(False)

        #Wait until exit event is published from etasl controller (/"controller_name"/e_event)
        rospy.wait_for_message('/etasl_controller_cmd/e_event', String)  
        self.counter += 1
        return "pickUpLineUp_complete"

# Nothing executed in this state. 
# Only a contianer for visualization of eTaSL states in smashviz
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
        smach.State.__init__(self, outcomes=["gripper_closed","gripper_opened","retry_gripper"])
        self.counter = 1
        self.is_open = True

    def execute(self, userdata):
        # OPEN/CLOSE GRIPPER (IF SIMULATION, WAIT 1s)
        if rospy.get_param("connection"):
            if not activate_gripper(self.is_open):
                return "retry_gripper"       
        else:
            rospy.sleep(1.0)
        if self.is_open:
            if self.counter == 1:
                peg_posititon(1,0,0,0,0)
            elif self.counter == 2:
                peg_posititon(2,1,0,0,0)
            elif self.counter == 3:
                peg_posititon(2,2,1,0,0)
            elif self.counter == 4:
                peg_posititon(2,2,2,1,0)
            elif self.counter == 5:
                peg_posititon(2,2,2,2,1)
            # ACTIVATING NEXT GROUP IN eTaSL
            activate_cmd_switch("+global.insertion_lineup_"+str(self.counter))
            self.is_open = False
            return "gripper_closed"
        else:
            if self.counter == 1:
                peg_posititon(2,0,0,0,0)
            elif self.counter == 2:
                peg_posititon(2,2,0,0,0)
            elif self.counter == 3:
                peg_posititon(2,2,2,0,0)
            elif self.counter == 4:
                peg_posititon(2,2,2,2,0)
            elif self.counter == 5:
                peg_posititon(2,2,2,2,2)   
            # ACTIVATING NEXT GROUP IN eTaSL         
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

# Nothing executed in this state. 
# Only a contianer for visualization of eTaSL states in smashviz
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
        rospy.sleep(5.0)
        stop()
        return "home_reached"        


if __name__ == "__main__":
    rospy.init_node("smach_peg")
    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, gripper_status)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["fully_complete"])
    with sm:
        smach.StateMachine.add("TakePicture", TakePicture(),
                               transitions={"picture_taken": "PickUpLineUp",
                                            "retry_picture": "TakePicture"})
        smach.StateMachine.add("PickUpLineUp", PickUpLineUp(),
                               transitions={"pickUpLineUp_complete": "PickUp"})
        smach.StateMachine.add("PickUp", PickUp(),
                               transitions={"pickUp_complete": "Gripper"}) 
        smach.StateMachine.add("Gripper", Gripper(),
                               transitions={"gripper_closed": "InsertionLineUp",
                                            "gripper_opened": "InsertionLineUp",
                                            "retry_gripper": "Gripper"}) 
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
        
