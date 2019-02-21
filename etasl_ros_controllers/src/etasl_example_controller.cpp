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

  if (!configureInput(node_handle))
  {
    return false;
  }

  if (!configureOutput(node_handle))
  {
    return false;
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
  if (etasl_->initialize(joint_position_map_, 10.0, 0.004, 1E-4, converged_values_map) < 0)
  {
    ROS_ERROR_STREAM("ExampleController: Could not initialize the eTaSl solver");
  }
}

void ExampleController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
  // Read from input channels
  getInput();

  // Read joint positions from hardware interface
  for (size_t i = 0; i < n_joints_; ++i)
  {
    joint_position_map_[joint_names_[i]] = position_joint_handles_[i].getPosition();
  }
  etasl_->setJointPos(joint_position_map_);

  // Solve the optimization problem
  etasl_->solve();

  // Get computed joint velocity, integrate, and set joint position command
  etasl_->getJointVel(joint_velocity_map_);
  for (size_t i = 0; i < n_joints_; ++i)
  {
    position_joint_handles_[i].setCommand(joint_position_map_[joint_names_[i]] +
                                          joint_velocity_map_[joint_names_[i]] * period.toSec());
  }

  // Write to output channels
  setOutput();
}

bool ExampleController::configureInput(ros::NodeHandle& node_handle)
{
  if (node_handle.getParam("/etasl/input/names", input_names_) &&
      node_handle.getParam("/etasl/input/types", input_types_))
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
        boost::function<void(const std_msgs::Float64ConstPtr&)> callback =
            [input_buffer](const std_msgs::Float64ConstPtr& msg) { input_buffer->writeFromNonRT(msg->data); };
        subs_.push_back(node_handle.subscribe<std_msgs::Float64>(input_names_[i], 1, callback));
        scalar_input_buffers_.push_back(input_buffer);
        ++n_scalar_inputs_;
      }
      else if (input_types_[i] == "Vector")
      {
        ROS_INFO_STREAM("ExampleController: Adding input channel \"" << input_names_[i] << "\" of type \"Vector\"");
        vector_input_names_.push_back(input_names_[i]);
        auto input_buffer = boost::make_shared<realtime_tools::RealtimeBuffer<geometry_msgs::Point>>();
        boost::function<void(const geometry_msgs::PointConstPtr&)> callback =
            [input_buffer](const geometry_msgs::PointConstPtr& msg) { input_buffer->writeFromNonRT(*msg); };
        subs_.push_back(node_handle.subscribe<geometry_msgs::Pose>(input_names_[i], 1, callback));
        vector_input_buffers_.push_back(input_buffer);
        ++n_vector_inputs_;
      }
      else if (input_types_[i] == "Rotation")
      {
        ROS_INFO_STREAM("ExampleController: Adding input channel \"" << input_names_[i] << "\" of type \"Rotation\"");
        rotation_input_names_.push_back(input_names_[i]);
        auto input_buffer = boost::make_shared<realtime_tools::RealtimeBuffer<geometry_msgs::Quaternion>>();
        boost::function<void(const geometry_msgs::QuaternionConstPtr&)> callback =
            [input_buffer](const geometry_msgs::QuaternionConstPtr& msg) { input_buffer->writeFromNonRT(*msg); };
        subs_.push_back(node_handle.subscribe<geometry_msgs::Quaternion>(input_names_[i], 1, callback));
        rotation_input_buffers_.push_back(input_buffer);
        ++n_rotation_inputs_;
      }
      else if (input_types_[i] == "Frame")
      {
        ROS_INFO_STREAM("ExampleController: Adding input channel \"" << input_names_[i] << "\" of type \"Frame\"");
        frame_input_names_.push_back(input_names_[i]);
        auto input_buffer = boost::make_shared<realtime_tools::RealtimeBuffer<geometry_msgs::Pose>>();
        boost::function<void(const geometry_msgs::PoseConstPtr&)> callback =
            [input_buffer](const geometry_msgs::PoseConstPtr& msg) { input_buffer->writeFromNonRT(*msg); };
        subs_.push_back(node_handle.subscribe<geometry_msgs::Pose>(input_names_[i], 1, callback));
        frame_input_buffers_.push_back(input_buffer);
        ++n_frame_inputs_;
      }
      else if (input_types_[i] == "Twist")
      {
        ROS_INFO_STREAM("ExampleController: Adding input channel \"" << input_names_[i] << "\" of type \"Twist\"");
        twist_input_names_.push_back(input_names_[i]);
        auto input_buffer = boost::make_shared<realtime_tools::RealtimeBuffer<geometry_msgs::Twist>>();
        boost::function<void(const geometry_msgs::TwistConstPtr&)> callback =
            [input_buffer](const geometry_msgs::TwistConstPtr& msg) { input_buffer->writeFromNonRT(*msg); };
        subs_.push_back(node_handle.subscribe<geometry_msgs::Twist>(input_names_[i], 1, callback));
        twist_input_buffers_.push_back(input_buffer);
        ++n_twist_inputs_;
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
  return true;
}

void ExampleController::getInput()
{
  // Read inputs
  if (n_scalar_inputs_ > 0)
  {
    for (size_t i = 0; i < n_scalar_inputs_; i++)
    {
      scalar_input_map_["global." + scalar_input_names_[i]] = *scalar_input_buffers_[i]->readFromRT();
    }
    etasl_->setInput(scalar_input_map_);
  }

  if (n_vector_inputs_ > 0)
  {
    for (size_t i = 0; i < n_vector_inputs_; i++)
    {
      Vector vector;
      tf::pointMsgToKDL(*vector_input_buffers_[i]->readFromRT(), vector);
      vector_input_map_["global." + vector_input_names_[i]] = vector;
    }
    etasl_->setInput(vector_input_map_);
  }

  if (n_rotation_inputs_ > 0)
  {
    for (size_t i = 0; i < n_rotation_inputs_; i++)
    {
      Rotation rotation;
      tf::quaternionMsgToKDL(*rotation_input_buffers_[i]->readFromRT(), rotation);
      rotation_input_map_["global." + rotation_input_names_[i]] = rotation;
    }
    etasl_->setInput(rotation_input_map_);
  }

  if (n_frame_inputs_ > 0)
  {
    for (size_t i = 0; i < n_frame_inputs_; i++)
    {
      Frame frame;
      tf::poseMsgToKDL(*frame_input_buffers_[i]->readFromRT(), frame);
      frame_input_map_["global." + frame_input_names_[i]] = frame;
    }
    etasl_->setInput(frame_input_map_);
  }

  if (n_twist_inputs_ > 0)
  {
    for (size_t i = 0; i < n_twist_inputs_; i++)
    {
      Twist twist;
      tf::twistMsgToKDL(*twist_input_buffers_[i]->readFromRT(), twist);
      twist_input_map_["global." + twist_input_names_[i]] = twist;
    }
    etasl_->setInput(twist_input_map_);
  }
}

bool ExampleController::configureOutput(ros::NodeHandle& node_handle)
{
  if (node_handle.getParam("/etasl/output/names", output_names_) &&
      node_handle.getParam("/etasl/output/types", output_types_))
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
        ROS_INFO_STREAM("ExampleController: Adding output channel \"" << output_names_[i] << "\" of type \"Scalar\"");
        scalar_output_names_.push_back(output_names_[i]);
        scalar_realtime_pubs_.push_back(
            boost::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(node_handle, output_names_[i], 4));
        ++n_scalar_outputs_;
      }
      else if (output_types_[i] == "Vector")
      {
        ROS_INFO_STREAM("ExampleController: Adding output channel \"" << output_names_[i] << "\" of type \"Vector\"");
        vector_output_names_.push_back(output_names_[i]);
        vector_realtime_pubs_.push_back(boost::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Point>>(
            node_handle, output_names_[i], 4));
        ++n_vector_outputs_;
      }
      else if (output_types_[i] == "Rotation")
      {
        ROS_INFO_STREAM("ExampleController: Adding output channel \"" << output_names_[i] << "\" of type \"Rotation\"");
        rotation_output_names_.push_back(output_names_[i]);
        rotation_realtime_pubs_.push_back(
            boost::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Quaternion>>(node_handle,
                                                                                             output_names_[i], 4));
        ++n_rotation_outputs_;
      }
      else if (output_types_[i] == "Frame")
      {
        ROS_INFO_STREAM("ExampleController: Adding output channel \"" << output_names_[i] << "\" of type \"Frame\"");
        frame_output_names_.push_back(output_names_[i]);
        frame_realtime_pubs_.push_back(boost::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Pose>>(
            node_handle, output_names_[i], 4));
        ++n_frame_outputs_;
      }
      else if (output_types_[i] == "Twist")
      {
        ROS_INFO_STREAM("ExampleController: Adding output channel \"" << output_names_[i] << "\" of type \"Twist\"");
        twist_output_names_.push_back(output_names_[i]);
        twist_realtime_pubs_.push_back(boost::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>(
            node_handle, output_names_[i], 4));
        ++n_twist_outputs_;
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
}

void ExampleController::setOutput()
{
  if (n_scalar_outputs_ > 0)
  {
    etasl_->getOutput(scalar_output_map_);
    for (size_t i = 0; i < n_scalar_outputs_; i++)
    {
      if (scalar_realtime_pubs_[i]->trylock())
      {
        scalar_realtime_pubs_[i]->msg_.data = scalar_output_map_["global." + scalar_output_names_[i]];
        scalar_realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }

  if (n_vector_outputs_ > 0)
  {
    etasl_->getOutput(vector_output_map_);
    for (size_t i = 0; i < n_vector_outputs_; i++)
    {
      if (vector_realtime_pubs_[i]->trylock())
      {
        Vector vector = vector_output_map_["global." + vector_output_names_[i]];
        tf::pointKDLToMsg(vector, vector_realtime_pubs_[i]->msg_);
        vector_realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }

  if (n_rotation_outputs_ > 0)
  {
    etasl_->getOutput(rotation_output_map_);
    for (size_t i = 0; i < n_rotation_outputs_; i++)
    {
      if (rotation_realtime_pubs_[i]->trylock())
      {
        Rotation rotation = rotation_output_map_["global." + rotation_output_names_[i]];
        tf::quaternionKDLToMsg(rotation, rotation_realtime_pubs_[i]->msg_);
        rotation_realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }

  if (n_frame_outputs_ > 0)
  {
    etasl_->getOutput(frame_output_map_);
    for (size_t i = 0; i < n_frame_outputs_; i++)
    {
      if (frame_realtime_pubs_[i]->trylock())
      {
        Frame frame = frame_output_map_["global." + frame_output_names_[i]];
        tf::poseKDLToMsg(frame, frame_realtime_pubs_[i]->msg_);
        frame_realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }

  if (n_twist_outputs_ > 0)
  {
    etasl_->getOutput(twist_output_map_);
    for (size_t i = 0; i < n_twist_outputs_; i++)
    {
      if (twist_realtime_pubs_[i]->trylock())
      {
        Twist twist = twist_output_map_["global." + twist_output_names_[i]];
        tf::twistKDLToMsg(twist, twist_realtime_pubs_[i]->msg_);
        twist_realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }
}

}  // namespace etasl_ros_controllers

PLUGINLIB_EXPORT_CLASS(etasl_ros_controllers::ExampleController, controller_interface::ControllerBase)