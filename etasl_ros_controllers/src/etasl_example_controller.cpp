// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the LGPL-3.0 license, see LICENSE
#include <etasl_ros_controllers/etasl_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace etasl_ros_controllers
{
bool ExampleController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr)
  {
    ROS_ERROR("ExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  if (!node_handle.getParam("/controller_joint_names", joint_names_))
  {
    ROS_ERROR("ExampleController: Could not parse joint names");
  }
  if (joint_names_.size() != 6)
  {
    ROS_ERROR_STREAM("ExampleController: Wrong number of joint names, got " << joint_names_.size()
                                                                            << " instead of 6 names!");
    return false;
  }
  position_joint_handles_.resize(6);
  for (size_t i = 0; i < 6; ++i)
  {
    try
    {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names_[i]);
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("ExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  if (!node_handle.getParam("/etasl/task_specificaton/file", task_specification_filename_))
  {
    ROS_ERROR("ExampleController: Could not find task specification filename");
  }

  etasl_ = boost::make_shared<EtaslDriver>(300, 0.0, 0.0001);

  return true;
}

void ExampleController::starting(const ros::Time& /* time */)
{
  for (size_t i = 0; i < 6; ++i)
  {
    initial_pos_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);

  etasl_->readTaskSpecificationFile(task_specification_filename_);

  DoubleMap initial_position_map;
  std::transform(joint_names_.begin(), joint_names_.end(), initial_pos_.begin(),
                 std::inserter(initial_position_map, initial_position_map.end()),
                 [](std::string a, double b) { return std::make_pair(a, b); });

  DoubleMap converged_values_map;
  etasl_->initialize(initial_position_map, 3.0, 0.004, 1E-4, converged_values_map);
}

void ExampleController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
  elapsed_time_ += period;

  std::array<double, 6> position;
  for (size_t i = 0; i < 6; ++i)
  {
    position[i] = position_joint_handles_[i].getPosition();
  }

  DoubleMap position_map;
  std::transform(joint_names_.begin(), joint_names_.end(), position.begin(),
                 std::inserter(position_map, position_map.end()),
                 [](std::string a, double b) { return std::make_pair(a, b); });

  DoubleMap input_map;
  double f1 = 1.0;
  double f2 = 2.5;
  input_map["tgt_x"] = sin(f1 * elapsed_time_.toSec()) * 0.15 + 0.7;
  input_map["tgt_y"] = sin(f2 * elapsed_time_.toSec()) * 0.1 + 0.4;
  input_map["tgt_z"] = 0.0;

  etasl_->setInput(input_map);

  etasl_->setJointPos(position_map);
  etasl_->solve();

  DoubleMap velocity_map;
  etasl_->getJointVel(velocity_map);

  for (size_t i = 0; i < 6; ++i)
  {
    position_joint_handles_[i].setCommand(position[i] + velocity_map[joint_names_[i]] * period.toSec());
  }

  DoubleMap output_map{};
  // output_map["error_x"] = 0.0;
  // output_map["error_y"] = 0.0;
  // output_map["error_z"] = 0.0;
  // StringVector output_names;
  // etasl_->getOutputNames(output_names);
  etasl_->getOutput(output_map);

  ROS_INFO_STREAM("error_x: " << output_map["global.error_x"]);
  // ROS_INFO_STREAM("error_y: " << output_map["global.error_y"]);
  // ROS_INFO_STREAM("error_z: " << output_map["global.error_z"]);
}

}  // namespace etasl_ros_controllers

PLUGINLIB_EXPORT_CLASS(etasl_ros_controllers::ExampleController, controller_interface::ControllerBase)