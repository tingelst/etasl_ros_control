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

  if (!node_handle.getParam("/etasl/task_specification", task_specification_))
  {
    ROS_ERROR("ExampleController: Could not find task specification on parameter server");
  }

  etasl_ = boost::make_shared<EtaslDriver>(300, 0.0, 0.0001);
  etasl_->readTaskSpecificationString(task_specification_);

  StringVector input_names{};
  etasl_->getInputNames(input_names);
  ROS_INFO_STREAM("ExampleController: Input channels:");
  for (auto name : input_names)
  {
    ROS_INFO_STREAM("\t" << name);
    name.erase(0, 7);  // Remove substring "global."
    auto input_buffer = boost::make_shared<realtime_tools::RealtimeBuffer<double>>();
    boost::function<void(const std_msgs::Float64ConstPtr&)> callback =
        [input_buffer](const std_msgs::Float64ConstPtr& msg) { input_buffer->writeFromNonRT(msg->data); };
    subs_.push_back(node_handle.subscribe<std_msgs::Float64>(name, 1, callback));

    input_buffers_.push_back(input_buffer);
  }

  StringVector output_names{};
  etasl_->getOutputNames(output_names);
  ROS_INFO_STREAM("ExampleController: Output channels:");
  for (auto name : output_names)
  {
    ROS_INFO_STREAM("\t" << name);
    name.erase(0, 7);  // Remove substring "global."
    realtime_pubs_.push_back(
        boost::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(node_handle, name, 4));
  }

  return true;
}

void ExampleController::starting(const ros::Time& /* time */)
{
  for (size_t i = 0; i < 6; ++i)
  {
    initial_pos_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);

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

  StringVector input_names{};
  etasl_->getInputNames(input_names);
  DoubleMap input_map;
  for (size_t i = 0; i < input_buffers_.size(); i++)
  {
    input_map[input_names[i]] = *input_buffers_[i]->readFromNonRT();
  }

  etasl_->setInput(input_map);

  etasl_->setJointPos(position_map);
  etasl_->solve();

  DoubleMap velocity_map;
  etasl_->getJointVel(velocity_map);

  for (size_t i = 0; i < 6; ++i)
  {
    position_joint_handles_[i].setCommand(position[i] + velocity_map[joint_names_[i]] * period.toSec());
  }

  StringVector output_names;
  etasl_->getOutputNames(output_names);
  DoubleMap output_map{};
  etasl_->getOutput(output_map);
  for (size_t i = 0; i < output_names.size(); i++)
  {
    if (realtime_pubs_[i]->trylock())
    {
      realtime_pubs_[i]->msg_.data = output_map[output_names[i]];
      realtime_pubs_[i]->unlockAndPublish();
    }
  }
}

}  // namespace etasl_ros_controllers

PLUGINLIB_EXPORT_CLASS(etasl_ros_controllers::ExampleController, controller_interface::ControllerBase)