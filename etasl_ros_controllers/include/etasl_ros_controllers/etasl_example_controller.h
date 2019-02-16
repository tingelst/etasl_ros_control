// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the LGPL-3.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/package.h>
#include <ros/node_handle.h>
#include <ros/time.h>

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
  bool initializeFeatureVariables(Context::Ptr ctx, solver& solver, double initialization_time, double sample_time, double convergence_crit);

  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 6> initial_pose_{};

  boost::shared_ptr<EtaslDriver> etasl_;
};
}  // namespace etasl_ros_controllers