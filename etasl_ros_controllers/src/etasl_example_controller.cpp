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

  if (!node_handle.getParam("/etasl/joint_names", joint_names_))
  {
    ROS_ERROR("ExampleController: Could not parse joint names");
  }
  n_joints_ = joint_names_.size();
  joint_position_.resize(n_joints_);
  position_joint_handles_.resize(n_joints_);
  for (size_t i = 0; i < n_joints_; ++i)
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

  if (node_handle.getParam("/etasl/input/names", input_names_) && node_handle.getParam("/etasl/input/types", input_types_))
  {
    if (!(input_names_.size() == input_types_.size()))
    {
      ROS_ERROR_STREAM("ExampleController: The number of input names and input types must be the same");
      return false;
    }
    n_inputs_ = input_names_.size();
    ROS_INFO_STREAM("ExampleController: Found " << n_inputs_ << " input channels");

    for (size_t i = 0; i < n_inputs_; ++i)
    {
      if (input_types_[i] == "scalar")
      {
        auto input_buffer = boost::make_shared<realtime_tools::RealtimeBuffer<double>>();
        boost::function<void(const std_msgs::Float64ConstPtr&)> callback = [input_buffer](const std_msgs::Float64ConstPtr& msg) {
          input_buffer->writeFromNonRT(msg->data);
        };
        subs_.push_back(node_handle.subscribe<std_msgs::Float64>(input_names_[i], 1, callback));
        scalar_input_buffers_.push_back(input_buffer);
      }
      else
      {
        ROS_ERROR_STREAM("ExampleController: Input channel type \"" << input_types_[i] << "\" is not supported");
        return false;
      }
    }
  }
  else
  {
    ROS_INFO("ExampleController: Could not find input channels on parameter server");
  }

  if (node_handle.getParam("/etasl/output/names", output_names_) && node_handle.getParam("/etasl/output/types", output_types_))
  {
    if (!(output_names_.size() == output_types_.size()))
    {
      ROS_ERROR_STREAM("ExampleController: The number of output names and output types must be the same");
      return false;
    }
    n_outputs_ = output_names_.size();
    ROS_INFO_STREAM("ExampleController: Found " << n_outputs_ << " output channels");

    for (size_t i = 0; i < n_outputs_; ++i)
    {
      if (output_types_[i] == "scalar")
      {
        realtime_pubs_.push_back(boost::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(node_handle, output_names_[i], 4));
      }
      else
      {
        ROS_ERROR_STREAM("ExampleController: Output channel type \"" << output_types_[i] << "\" is not supported");
        return false;
      }
    }
  }
  else
  {
    ROS_INFO("ExampleController: Could not find output channels on parameter server");
  }

  if (!node_handle.getParam("/etasl/task_specification", task_specification_))
  {
    ROS_ERROR("ExampleController: Could not find task specification on parameter server");
    return false;
  }
  etasl_ = boost::make_shared<EtaslDriver>(300, 0.0, 0.0001);
  etasl_->readTaskSpecificationString(task_specification_);

  return true;
}

void ExampleController::starting(const ros::Time& /* time */)
{
  for (size_t i = 0; i < 6; ++i)
  {
    initial_pos_[i] = position_joint_handles_[i].getPosition();
  }

  DoubleMap initial_position_map;
  std::transform(joint_names_.begin(), joint_names_.end(), initial_pos_.begin(), std::inserter(initial_position_map, initial_position_map.end()),
                 [](std::string a, double b) { return std::make_pair(a, b); });

  DoubleMap converged_values_map;
  etasl_->initialize(initial_position_map, 3.0, 0.004, 1E-4, converged_values_map);
}

void ExampleController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
  DoubleMap joint_position_map;
  for (size_t i = 0; i < n_joints_; ++i)
  {
    joint_position_[i] = position_joint_handles_[i].getPosition();
    joint_position_map[joint_names_[i]] = joint_position_[i];
  }
  etasl_->setJointPos(joint_position_map);

  DoubleMap input_map;
  for (size_t i = 0; i < n_inputs_; i++)
  {
    input_map["global." + input_names_[i]] = *scalar_input_buffers_[i]->readFromNonRT();
  }
  etasl_->setInput(input_map);

  etasl_->solve();

  DoubleMap velocity_map;
  etasl_->getJointVel(velocity_map);

  for (size_t i = 0; i < 6; ++i)
  {
    position_joint_handles_[i].setCommand(joint_position_[i] + velocity_map[joint_names_[i]] * period.toSec());
  }

  StringVector output_names;
  etasl_->getOutputNames(output_names);
  DoubleMap output_map{};
  etasl_->getOutput(output_map);

  for (size_t i = 0; i < n_outputs_; i++)
  {
    if (realtime_pubs_[i]->trylock())
    {
      realtime_pubs_[i]->msg_.data = output_map["global." + output_names_[i]];
      realtime_pubs_[i]->unlockAndPublish();
    }
  }
}

}  // namespace etasl_ros_controllers

PLUGINLIB_EXPORT_CLASS(etasl_ros_controllers::ExampleController, controller_interface::ControllerBase)