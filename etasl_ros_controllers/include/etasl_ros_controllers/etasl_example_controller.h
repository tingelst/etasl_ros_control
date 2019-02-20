// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the LGPL-3.0 license, see LICENSE
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

#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <expressiongraph/context.hpp>
#include <expressiongraph/qpoases_solver.hpp>
#include <expressiongraph/context_scripting.hpp>

#include <etasl_ros_controllers/etasl_driver.h>

using namespace KDL;

namespace etasl_ros_controllers
{
class ExampleController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;

  std::array<double, 6> initial_pos_{};

  std::vector<double> joint_position_;

  std::vector<std::string> joint_names_;
  size_t n_joints_{};

  std::vector<std::string> input_names_;
  std::vector<std::string> input_types_;
  size_t n_inputs_{};
  std::vector<boost::shared_ptr<realtime_tools::RealtimeBuffer<double>>> scalar_input_buffers_;

  std::vector<std::string> output_names_;
  std::vector<std::string> output_types_;
  size_t n_outputs_{};

  std::string task_specification_;
  boost::shared_ptr<EtaslDriver> etasl_;

  std::vector<boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>>> realtime_pubs_;
  std::vector<ros::Subscriber> subs_;
  std::vector<boost::shared_ptr<realtime_tools::RealtimeBuffer<double>>> input_buffers_;
};
}  // namespace etasl_ros_controllers