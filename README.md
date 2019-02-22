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

## Download and build `etasl_ros_control`
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

## Source the catkin workspace
Setup your environment - you can do this every time you work with this particular source install of the code, or you can add this to your `.bashrc` (recommended):
```bash
source ~/etasl_ws/devel/setup.bash # or .zsh, depending on your shell
```

## Acknowledgements

- The expressiongraph and eTaSL projects has been developed by KU Leuven.
- The above installation instructions are modified from the MoveIt! installation instructions found [here](https://moveit.ros.org/install/source/).
