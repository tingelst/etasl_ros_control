#!/usr/bin/env python
"""Simple example using services to switch etasl controllers and SMACH for state machine handling."""
import rospy
import smach
import smach_ros


from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchControllerResponse
from controller_manager_msgs.srv import SwitchControllerRequest


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


# Define the laser "etching" operational mode
class Etch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["single_etching_complete",
                                             "10_etchings_complete"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("------------Executing etch operation.")
        sane_controller_switching("etasl_controller")
        rospy.sleep(20.)  # Sleep for 20 seconds.
        # Should be handled with monitors
        self.counter += 1
        if self.counter > 10:
            return "10_etchings_complete"
        else:
            return "single_etching_complete"


# Define the "Stay at home" state. Staying still while the paper to be etched
# is removed.
class Home(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["wait_complete"])

    def execute(self, userdata):
        rospy.loginfo("---------Staying home.")
        sane_controller_switching("etasl_controller_2")
        rospy.sleep(20.)
        return "wait_complete"


if __name__ == "__main__":
    rospy.init_node("smach_example")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["fully_complete"])
    with sm:
        smach.StateMachine.add("Etch", Etch(),
                               transitions={
                                   "single_etching_complete": "Home",
                                   "10_etchings_complete": "fully_complete"})
        smach.StateMachine.add("Home", Home(),
                               transitions={"wait_complete": "Etch"})
    sis = smach_ros.IntrospectionServer('laser_etching', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        sis.stop()
        
