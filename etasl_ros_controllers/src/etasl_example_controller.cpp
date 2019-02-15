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
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("/controller_joint_names", joint_names))
  {
    ROS_ERROR("ExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 6)
  {
    ROS_ERROR_STREAM("ExampleController: Wrong number of joint names, got " << joint_names.size()
                                                                            << " instead of 6 names!");
    return false;
  }
  position_joint_handles_.resize(6);
  for (size_t i = 0; i < 6; ++i)
  {
    try
    {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("ExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  //   std::array<double, 7> q_start{ { 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4 } };
  //   for (size_t i = 0; i < q_start.size(); i++)
  //   {
  //     if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1)
  //     {
  //       ROS_ERROR_STREAM("JointPositionExampleController: Robot is not in the expected starting position for "
  //                        "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
  //                        "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
  //       return false;
  //     }
  //   }

  ctx_ = create_context();
  ctx_->addType("robot");
  ctx_->addType("feature");

  lua_ctx_ = boost::make_shared<LuaContext>();
  lua_ctx_->initContext(ctx_);

  std::string task_specification_filename;
  if (!node_handle.getParam("/etasl/task_specificaton/file", task_specification_filename))
  {
    ROS_ERROR("ExampleController: Could not find task specification filename");
  }
  if (lua_ctx_->executeFile(task_specification_filename) != 0)
  {
    ROS_ERROR_STREAM("ExampleController: Could not load task specification from file");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("ExampleController: Loaded task specification");
  }

  double max_iterations = floor(ctx_->getSolverProperty("max_iterations", 300));
  double max_cpu_time = ctx_->getSolverProperty("max_cpu_time", 0.0);
  double sample_time = ctx_->getSolverProperty("sample_time", 0.01);
  double regularization = ctx_->getSolverProperty("regularization", 1E-4);
  double initialization_time = ctx_->getSolverProperty("initialization_time", 3);

  solver_ = boost::make_shared<qpOASESSolver>(max_iterations, max_cpu_time, regularization);

  return true;
}

bool ExampleController::initializeFeatureVariables(Context::Ptr ctx, solver& solver, double initialization_time,
                                                   double sample_time, double convergence_crit)
{
  // RTT::os::TimeService::ticks start_time = RTT::os::TimeService::Instance()->getTicks();
  // initialization:
  if (solver_->getNrOfFeatureStates() > 0)
  {
    ROS_INFO_STREAM("ExampleController: initialization started");
    double t;
    for (t = 0; t < initialization_time; t += sample_time)
    {
      int retval = solver_->updateStep(sample_time);
      if (retval != 0)
      {
        ROS_ERROR_STREAM("ExampleController: Solver encountered the following error during initialization (t=" << t
                                                                                                               << " )");
        ROS_ERROR_STREAM(solver_->errorMessage(retval));
        ROS_ERROR_STREAM(ctx_);
        return false;
      }
      double norm_change = solver_->getNormChange() * sample_time;
      ROS_INFO_STREAM("ExampleController: norm change: " << norm_change);
      if (norm_change <= convergence_crit)
      {
        break;
      }
    }
    // double elapsed = RTT::os::TimeService::Instance()->secondsSince(start_time);
    // ROS_INFO_STREAM("ExampleController: Initialization time(" << ceil(t / sample_time) << "  iterations) : " <<
    // elapsed
    //                                                           << "[s]");
  }
  solver_->setInitialValues();  // sets the initial value fields of variables in the context.
  // solver.printMatrices(std::cout);
  return true;
}

void ExampleController::starting(const ros::Time& /* time */)
{
  for (size_t i = 0; i < 6; ++i)
  {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}

void ExampleController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
  elapsed_time_ += period;

  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;
  for (size_t i = 0; i < 6; ++i)
  {
    position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
  }
}

}  // namespace etasl_ros_controllers

PLUGINLIB_EXPORT_CLASS(etasl_ros_controllers::ExampleController, controller_interface::ControllerBase)