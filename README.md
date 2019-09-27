# etasl_ros_control

This project integrates **eTaSL** with **ROS Control**.

eTaSL is a task specification language for reactive control of robot systems developed by KU Leuven. It is a language that allows you to describe how your robotic system has to move and interact with sensors. This description is based on a constraint-based methodology. Everything is specified as an optimization problem subject to constraints.

More information on eTaSL can be found [here](https://rob-expressiongraphs.pages.mech.kuleuven.be/expressiongraph_doc/intro.html).


## Build `etasl_ros_control` from source.

For now, `etasl_ros_control` is only supported on Linux:
- Ubuntu 16.04 / ROS Kinetic

Follow all the instructions to [install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). Please make sure you have followed all steps and have the latest versions of packages installed:
```bash 
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
```
Source installation requires wstool:
```bash 
sudo apt-get install python-wstool
```
Optionally create a new workspace, you can name it whatever:
```bash
mkdir ~/etasl_ros_control_ws
cd ~/etasl_ros_control_ws
```
Next, source your ROS workspace to load the necessary environment variables.
```bash
source /opt/ros/kinetic/setup.bash
```
This will load the ${ROS_DISTRO} variable, needed for the next step.

### Download and build `etasl_ros_control`
By default we will assume you are building on the latest branch, we currently use *master* as our master branch. 

Pull down the required repositories and build from within the root directory of your catkin workspace:
```bash
wstool init src
wstool merge -t src https://raw.githubusercontent.com/tingelst/etasl_ros_control/master/etasl_ros_control.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin_init_workspace src
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Source the catkin workspace
Setup your environment - you can do this every time you work with this particular source install of the code, or you can add this to your `.bashrc` (recommended):
```bash
source ~/etasl_ros_control_ws/devel/setup.bash # or .zsh, depending on your shell
```

## Getting started

If you haven’t already done so, make sure you’ve completed the steps above.

### Launch the examples

There are three `launch` files available: One using the KUKA KR6 R900 sixx Agilus robot, one using the UR10 robot with Gazebo, and one showing how to setup multiple controllers that each load a separate task specification.

Launch the example using the KUKA Agilus:
```bash
roslaunch etasl_ros_control_examples example_kuka_kr6r900sixx.launch
```

Launch the example using the UR10:
```bash
roslaunch etasl_ros_control_examples example_ur10_gazebo.launch
```

You can visualize the trajectory by adding a *Marker* in RViz: Add -> Marker -> OK

#### Multiple controllers

Launch the example using multiple controllers:
```bash 
roslaunch etasl_ros_control_examples example_multiple_controllers.launch
```
This example loads three controllers, one joint trajectory controller, and two eTaSL controllers. The joint trajectory controller (`position_trajectory_controller`) is started, while the two eTaSL controllers are loaded. To switch to the first eTaSL controller (`etasl_controller`) you can use:
```bash
rosservice call /controller_manager/switch_controller [etasl_controller] [position_trajectory_controller] 2
```
Then, to switch to the second eTaSL controller (`etasl_controller_2`), you can use 
```bash
rosservice call /controller_manager/switch_controller [etasl_controller_2] [etasl_controller] 2
```
And, finally back to the joint trajectory controller:
```bash
rosservice call /controller_manager/switch_controller [position_trajectory_controller] [etasl_controller_2] 2
```

### SMACH for discrete controller switching
The multiple controllers can be used in a [SMACH](https://wiki.ros.org/smach) example. 

Launch the smach example:
```
roslaunch etasl_ros_control_examples example_smach.launch
```

## Acknowledgements

- The expressiongraph and eTaSL projects have been developed by KU Leuven.
- The above installation instructions are modified from the MoveIt! installation instructions found [here](https://moveit.ros.org/install/source/).
