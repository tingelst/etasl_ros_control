#!/usr/bin/env python
"""Simple example using services to switch etasl controllers and SMACH for state machine handling."""
import rospy
import smach
import smach_ros
#import boost_etasl_driver

from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchControllerResponse
from controller_manager_msgs.srv import SwitchControllerRequest
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
from std_msgs.msg import String



def listener():
    rospy.wait_for_message('/etasl_controller/e_event', String)
    rospy.loginfo("Exit recived")
    

class PickUpLineUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["pickUpLineUp_complete"])
        self.counter = 1

    def execute(self, userdata):
        rospy.loginfo("Lining up for pickup %i",self.counter)
        #boost_etasl_driver.activation_command("+global.pickup_lineup_" + self.counter)

        listener()
        self.counter += 1
        return "pickUpLineUp_complete"


class PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["pickUp_complete"])
        self.counter = 1

    def execute(self, userdata):
        rospy.loginfo("Approaching peg %i",self.counter)
        #boost_etasl_driver.activation_command("+global.pickup_closein_" + self.counter)

        #Wait until exit event is published
        listener()

        rospy.loginfo("Closing gripper")
        action_name = rospy.get_param('~action_name', 'command_robotiq_action')
        robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
        # Wait until grippers are ready to take command
        robotiq_client.wait_for_server()
        Robotiq.close(robotiq_client, speed=0.5, force=255)

        return "pickUp_complete"

class InsertionLineUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["insertionLineUp_complete"])
        self.counter = 1

    def execute(self, userdata):
        rospy.loginfo("Lining up for insertion ",self.counter)
        #boost_etasl_driver.activation_command("-global.pickup_lineup_" + self.counter + " -global.pickup_closein_" + self.counter + " +global.insertion_lineup_" + self.counter)
        
        #sane_controller_switching("etasl_controller")
        #rospy.sleep(20.)  # Sleep for 20 seconds.
        # Should be handled with monitors

        self.counter += 1
        return "insertionLineUp_complete"

class InsertionCloseIn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["insertionCloseIn_complete",
                                             "high_force_registered",
                                             "all_pegs_placed"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Interting peg")
        #boost_etasl_driver.activation_command("+global.insertion_closein")

        self.counter += 1
        if self.counter > 5:
            return "high_force_registered"
        else: 
            return "insertionCloseIn_complete"

class Lissajous(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["insertion_complete"])

    def execute(self, userdata):
        rospy.loginfo("Activting lissajous")
        #boost_etasl_driver.activation_command("+global.lissajous")

        return "insertion_complete"

class Home(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["reached_home"])

    def execute(self, userdata):
        rospy.loginfo("Reached home position")
        #boost_etasl_driver.activation_command("+global.lissajous")

        return "reached_home"

if __name__ == "__main__":
    rospy.init_node("smach_peg_in_hole")
    #listener()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["fully_complete"])
    with sm:
        smach.StateMachine.add("PickUpLineUp", PickUpLineUp(),
                               transitions={"pickUpLineUp_complete": "PickUp"})
        smach.StateMachine.add("PickUp", PickUp(),
                               transitions={"pickUp_complete": "InsertionLineUp"})
        smach.StateMachine.add("InsertionLineUp", InsertionLineUp(),
                               transitions={"insertionLineUp_complete": "InsertionCloseIn"})
        smach.StateMachine.add("InsertionCloseIn", InsertionCloseIn(),
                               transitions={"insertionCloseIn_complete": "PickUpLineUp",
                                            "high_force_registered": "Lissajous",
                                            "all_pegs_placed": "Home"})
        smach.StateMachine.add("Lissajous", Lissajous(),
                               transitions={"insertion_complete": "PickUpLineUp"})
        smach.StateMachine.add("Home", Home(),
                               transitions={"reached_home": "fully_complete"})                                                        
    sis = smach_ros.IntrospectionServer('peg_in_hole', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        sis.stop()
        
