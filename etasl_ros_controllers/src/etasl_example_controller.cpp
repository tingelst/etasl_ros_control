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
      if (input_types_[i] == "Scalar")
      {
        ROS_INFO_STREAM("ExampleController: Adding input channel \"" << input_names_[i] << "\" of type \"Scalar\"");
        scalar_input_names_.push_back(input_names_[i]);
        auto input_buffer = boost::make_shared<realtime_tools::RealtimeBuffer<double>>();
        boost::function<void(const std_msgs::Float64ConstPtr&)> callback = [input_buffer](const std_msgs::Float64ConstPtr& msg) {
          input_buffer->writeFromNonRT(msg->data);
        };
        subs_.push_back(node_handle.subscribe<std_msgs::Float64>(input_names_[i], 1, callback));
        scalar_input_buffers_.push_back(input_buffer);
        ++n_scalar_inputs_;
      }
      else if (input_types_[i] == "Frame")
      {
        ROS_INFO_STREAM("ExampleController: Adding input channel \"" << input_names_[i] << "\" of type \"Frame\"");
        frame_input_names_.push_back(input_names_[i]);
        auto input_buffer = boost::make_shared<realtime_tools::RealtimeBuffer<Frame>>();
        boost::function<void(const geometry_msgs::PoseConstPtr&)> callback = [input_buffer](const geometry_msgs::PoseConstPtr& msg) {
          Frame frame;
          tf::poseMsgToKDL(*msg, frame);
          input_buffer->writeFromNonRT(frame);
        };
        subs_.push_back(node_handle.subscribe<geometry_msgs::Pose>(input_names_[i], 1, callback));
        frame_input_buffers_.push_back(input_buffer);
        ++n_frame_inputs_;
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
      if (output_types_[i] == "Scalar")
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
  for (size_t i = 0; i < n_joints_; ++i)
  {
    joint_position_map_[joint_names_[i]] = position_joint_handles_[i].getPosition();
  }

  DoubleMap converged_values_map;
  etasl_->initialize(joint_position_map_, 3.0, 0.004, 1E-4, converged_values_map);
}

void ExampleController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
  // Read joint positions from hardware interface
  for (size_t i = 0; i < n_joints_; ++i)
  {
    joint_position_map_[joint_names_[i]] = position_joint_handles_[i].getPosition();
  }
  etasl_->setJointPos(joint_position_map_);

  // Read inputs
  for (size_t i = 0; i < n_scalar_inputs_; i++)
  {
    scalar_input_map_["global." + scalar_input_names_[i]] = *scalar_input_buffers_[i]->readFromRT();
  }
  etasl_->setInput(scalar_input_map_);

  for (size_t i = 0; i < n_frame_inputs_; i++)
  {
    frame_input_map_["global." + frame_input_names_[i]] = *frame_input_buffers_[i]->readFromRT();
  }
  etasl_->setInput(frame_input_map_);

  // Solve the optimization problem
  etasl_->solve();

  // Get computed joint velocity, integrate, and set joint position command
  etasl_->getJointVel(joint_velocity_map_);
  for (size_t i = 0; i < n_joints_; ++i)
  {
    position_joint_handles_[i].setCommand(joint_position_map_[joint_names_[i]] + joint_velocity_map_[joint_names_[i]] * period.toSec());
  }

  // Write outputs
  etasl_->getOutput(output_map_);
  for (size_t i = 0; i < n_outputs_; i++)
  {
    if (realtime_pubs_[i]->trylock())
    {
      realtime_pubs_[i]->msg_.data = output_map_["global." + output_names_[i]];
      realtime_pubs_[i]->unlockAndPublish();
    }
  }
}  // namespace etasl_ros_controllers

}  // namespace etasl_ros_controllers

PLUGINLIB_EXPORT_CLASS(etasl_ros_controllers::ExampleController, controller_interface::ControllerBase)