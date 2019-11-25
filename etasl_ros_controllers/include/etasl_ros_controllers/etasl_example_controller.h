// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the LGPL-3.0 license, see LICENSE

/*
 * Author: Lars Tingelstad
 */

#pragma once

#include <array>
#include <string>
#include <vector>

#include <boost/function.hpp>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/package.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <kdl_conversions/kdl_msg.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <expressiongraph/context.hpp>
#include <expressiongraph/qpoases_solver.hpp>
#include <expressiongraph/context_scripting.hpp>

#include <etasl_ros_controllers/etasl_driver.h>
#include <etasl_ros_controllers/topic_observer.h>

using namespace KDL;

namespace etasl_ros_controllers
{
using FrameMap = std::map<std::string, Frame>;
using VectorMap = std::map<std::string, Vector>;
using RotationMap = std::map<std::string, Rotation>;
using TwistMap = std::map<std::string, Twist>;
using WrenchMap = std::map<std::string, Wrench>;

class EtaslController
  : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;

  void solve();
  bool configureInput(ros::NodeHandle& node_handle);
  void getInput();
  void setOutput();
  bool configureOutput(ros::NodeHandle& node_handle);

  DoubleMap joint_position_map_;
  std::vector<std::string> joint_names_;
  size_t n_joints_{};

  DoubleMap joint_velocity_map_;

  // Inputs
  std::vector<std::string> input_names_;
  std::vector<std::string> input_types_;
  size_t n_inputs_{};
  std::vector<ros::Subscriber> subs_;

  DoubleMap scalar_input_map_;
  size_t n_scalar_inputs_{};
  std::vector<std::string> scalar_input_names_;
  std::vector<boost::shared_ptr<realtime_tools::RealtimeBuffer<double>>> scalar_input_buffers_;

  VectorMap vector_input_map_;
  size_t n_vector_inputs_{};
  std::vector<std::string> vector_input_names_;
  std::vector<boost::shared_ptr<realtime_tools::RealtimeBuffer<geometry_msgs::Point>>> vector_input_buffers_;

  RotationMap rotation_input_map_;
  size_t n_rotation_inputs_{};
  std::vector<std::string> rotation_input_names_;
  std::vector<boost::shared_ptr<realtime_tools::RealtimeBuffer<geometry_msgs::Quaternion>>> rotation_input_buffers_;

  FrameMap frame_input_map_;
  size_t n_frame_inputs_{};
  std::vector<std::string> frame_input_names_;
  std::vector<boost::shared_ptr<realtime_tools::RealtimeBuffer<geometry_msgs::Pose>>> frame_input_buffers_;

  TwistMap twist_input_map_;
  size_t n_twist_inputs_{};
  std::vector<std::string> twist_input_names_;
  std::vector<boost::shared_ptr<realtime_tools::RealtimeBuffer<geometry_msgs::Twist>>> twist_input_buffers_;

  WrenchMap wrench_input_map_;
  size_t n_wrench_inputs_{};
  std::vector<std::string> wrench_input_names_;
  std::vector<boost::shared_ptr<realtime_tools::RealtimeBuffer<geometry_msgs::Wrench>>> wrench_input_buffers_;

  // Outputs
  DoubleMap output_map_;
  std::vector<std::string> output_names_;
  std::vector<std::string> output_types_;
  size_t n_outputs_{};

  std::vector<std::string> scalar_output_names_;
  DoubleMap scalar_output_map_;
  size_t n_scalar_outputs_{};
  std::vector<boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> scalar_realtime_pubs_;

  std::vector<std::string> vector_output_names_;
  VectorMap vector_output_map_;
  size_t n_vector_outputs_{};
  std::vector<boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Point>>> vector_realtime_pubs_;

  std::vector<std::string> rotation_output_names_;
  RotationMap rotation_output_map_;
  size_t n_rotation_outputs_{};
  std::vector<boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Quaternion>>> rotation_realtime_pubs_;

  std::vector<std::string> frame_output_names_;
  FrameMap frame_output_map_;
  size_t n_frame_outputs_{};
  std::vector<boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Pose>>> frame_realtime_pubs_;

  std::vector<std::string> twist_output_names_;
  TwistMap twist_output_map_;
  size_t n_twist_outputs_{};
  std::vector<boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>>> twist_realtime_pubs_;

  boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::String>> event_pub_p_;

  std::string task_specification_;
  boost::shared_ptr<EtaslDriver> etasl_;
};
}  // namespace etasl_ros_controllers
