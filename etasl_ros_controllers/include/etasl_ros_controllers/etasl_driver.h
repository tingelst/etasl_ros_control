// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the LGPL-3.0 license, see LICENSE

/*
 * Author: Lars Tingelstad
 *
 * This file has been adapted from:
 * https://gitlab.mech.kuleuven.be/rob-expressiongraphs/etasl_py/blob/master/include/etasl_py/etaslcppdriver.hpp
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <chrono>

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include <expressiongraph/context.hpp>
#include <expressiongraph/qpoases_solver.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/defaultobserver.hpp>
#include <expressiongraph/outputs_ros_lines.hpp>

#include <ros/ros.h>

using namespace KDL;

namespace etasl_ros_controllers
{
typedef std::map<std::string, double> DoubleMap;
typedef std::map<std::string, Frame> FrameMap;
typedef std::vector<std::string> StringVector;
using VectorMap = std::map<std::string, Vector>;
using RotationMap = std::map<std::string, Rotation>;
using TwistMap = std::map<std::string, Twist>;
using WrenchMap = std::map<std::string, Wrench>;

class EtaslDriver
{
  boost::shared_ptr<qpOASESSolver> solver_;
  bool etaslread;
  bool initialized_;
  boost::shared_ptr<LuaContext> lua;
  boost::shared_ptr<Observer> obs_;
  boost::shared_ptr<OutputGenerator> out_;

  StringVector joint_names_;
  Eigen::VectorXd joint_values_;
  Eigen::VectorXd joint_velocities_;
  StringVector feature_names_;
  Eigen::VectorXd feature_values_;
  Eigen::VectorXd feature_velocities_;
  int time_ndx_;

  std::vector<int> all_ndx;  // tmp storage for requesting variable indices to context.

  /**
   * initializes the feature variables that are involved in the priority 0 constraints:
   * @param initialization_time:  maximum (virtual) duration of the initialization phase.
   * @param sample_time:          sample time at which to run the initialization
   * @param convergence_crit:     convergence criterion to stop the initialization.
   *
   * @returns: true if successful initialization, false otherwise.
   *   returns the values of the joint and feature variables in the result variable.
   */
  bool initializeFeatureVariables(double initialization_time, double sample_time, double convergence_crit,
                                  DoubleMap& result);

  bool initializeFeatureVariables(Context::Ptr ctx, solver& solver, double initialization_time, double sample_time,
                                  double convergence_crit);

public:
  boost::shared_ptr<Context> ctx_;
  /**
   * eTaSLCppDriver constructor
   *
   * @param  nWSR:                   maximum number of solver iterations.
   * @param  cputime:                maximum execution time. If this value is equal to zero,
   *                                 it is ignored.
   * @param regularisation_factor:   regularisation_factor to be used.
   *
   */
  EtaslDriver(int nWSR, double cputime, double regularization_factor);

  /**
   * sets all (scalar) variables specified in the map as input variable
   *
   * @param dmap:
   *    map of the names of the defined inputchannels in the eTaSL and the
   *     values to assign to these inputchannels.
   *
   * @returns 0 if successful, -1 in case of error (undefined inputchannel or
   *  inputchannel of wrong type).
   */
  int setInput(const DoubleMap& dmap);
  int setInput(const FrameMap& fmap);
  int setInput(const RotationMap& rmap);
  int setInput(const VectorMap& fmap);
  int setInput(const TwistMap& tmap);
  int setInput(const WrenchMap& tmap);

  /**
   * sets all (scalar) variables with the velocity specified in the map as input variable
   *
   * @param dmap:
   *    map of the names of the defined inputchannels in the eTaSL and the
   *     VELOCITY values to assign to these inputchannels.
   *
   * @returns 0 if successful, -1 in case of error (undefined inputchannel or
   *  inputchannel of wrong type).
   */
  int setInputVelocity(const DoubleMap& dmap);

  /**
   * sets all (scalar) (position) variables specified in the map
   * as controller or feature variable
   *
   * @param dmap:   map containing a variable name and its value
   * @returns number of variables that are filled in or -1 in case of
   *          error (when called before initialized() )
   *
   * for all robot and feature variables, checks whether the name is present in dmap, and
   * if so, fill in the value in the corresponding vector.  If a robot or feature variable
   * is not present in dmap, its value will remain unchanged.
   *
   * @warning: "time" is also given using this function
   *
   * @warning: non existing names are ignored
   *
   * @warning: in some scenario's you want to call this method multiple times,
   *  e.g. to fill in robot variables and feature variables separately
   */
  int setJointPos(const DoubleMap& dmap);

  int setJointPos(const std::vector<double>& joint_positions, std::vector<std::string>& joint_names);

  /**
   * gets the (velocity) output for the variables specified in the map (controller or
   * feature variables).
   *
   * @param dmap:  map that will contain the names of the variables and their value
   * @param flag:  if 1 only fills in the robot variables, if 2 only fill in the feature
   *               variables, if 3, fill in both types of variables.
   *               (default value=3)
   *
   * @returns: 0 if succesful, -1 when called before the etasl spec is read.
   */
  int getJointVel(DoubleMap& dmap, int flag = 3);

  /**
   * gets the position the variables specified in the map (controller or
   * feature variables).
   *
   * @param dmap:  map that will contain the names of the variables and their value
   * @param flag:  if 1 only fills in the robot variables, if 2 only fill in the feature
   *               variables, if 3, fill in both types of variables.
   *               (default value=3)
   *
   * @returns: 0 if succesful, -1 when called before the etasl spec is read.
   */
  int getJointPos(DoubleMap& dmap, int flag = 3);

  /**
   * gets all available (scalar) output variables
   */
  void getOutput(DoubleMap& dmap);
  void getOutput(VectorMap& fmap);
  void getOutput(RotationMap& rmap);
  void getOutput(FrameMap& fmap);
  void getOutput(TwistMap& tmap);

  /**
   * reads a task specification file and configures the controller accordingly.
   * can throw a LuaException()
   */
  void readTaskSpecificationFile(const std::string& filename);

  /**
   * reads a task specification string and configures the controller accordingly.
   * can throw a LuaException()
   */
  void readTaskSpecificationString(const std::string& taskspec);

  /**
   * Performs the following tasks in this order:
   *  1) prepares the solver for the initialization problem
   *  2) initializes the state (robot/feature names, values and velocities)
   *  3) sets the initial value for robot/feature variables
   *  4) performs an optimization to compute an optimal start value
   * for the feature variables (only taking into account the constraints with priority==0)
   *  5) prepares the solver for the exuction problem and solves one step
   *     (such that next steps are all hot-starts)
   *
   *  @param initialval: map containing the name and value of robot- and feature variables you
   *                     want to initialize (before initialization optimization)
   *  @param initialization_time:  max. (virtual) time to use for initialization
   *  @param sample_time:          (virtual) sample time to use for the initialization
   *  @param convergence_crit:     convergence criterion used to stop the initialization early.
   *  @param converged_val   :     will contain the converged values for joints and feature variables.
   *  @warning:
   *      - initializes time to 0, you can overwrite this in the initialval map.
   *      - robot variables remain identical as specified after this method call.
   *      - feature variables can be changed after this method call (if they are involved
   *        in the initialization optimization).
   *
   *  returns:
   *   -1 : context contains priority levels the solver can't handle (initialization problem)
   *   -2 : initialization optimization did not converge
   *   -3 : context contains priority levels the solver can't handle (execution problem)
   *   -4 : first run of the execution problem failed.
   *   -5 : no task specification present
   */
  int initialize(const DoubleMap& initialval, double initialization_time, double sample_time, double convergence_crit,
                 DoubleMap& converged_val);

  /**
   * Computes the controller output for 1 time step.
   * (reads out the task specification, computes the matrices of the
   *  optimization problem, solves the optimization problem)
   *
   * Each time solve() runs, the events will be cleared.
   * returns:
   *  -1: in case of fatal error
   *  1:  in case of a triggered event
   *  0:  no errors and no events
   */
  int solve();

  int updateStep(double dt);

  /**
   * evaluate everything in the task specification problem
   * (but without solving the optimization problem)
   */
  void evaluate();

  /**
   * Returns the triggered event
   */
  std::string getEvent();

  /**
   * the number of feature variables in the problem
   * returns -1 if error otherwise the requested number.
   */
  int nrOfFeatureVar();

  /**
   * the number of robot variables in the problem
   * returns -1 if error otherwise the requested number.
   */
  int nrOfRobotVar();

  /**
   * the number of constraints in the problem
   * returns -1 if error otherwise the requested number.
   */
  int nrOfScalarConstraints();

  /**
   * the number of box constraints in the problem
   * returns -1 if error otherwise the requested number.
   */
  int nrOfBoxConstraints();

  /**
   * requests the name, weight and initial value for the variables
   * declared in the context.
   *
   * flag == 1 for only robot variables,
   * flag == 2 for only feature variables,
   * flag == 3 for both.
   */
  void getVariables(int flag, std::vector<std::string>& name, std::vector<double>& weight,
                    std::vector<double>& initval);

  /**
   * request all declared input channels
   */
  void getInputNames(std::vector<std::string>& name);

  /**
   * request all declared output expressions
   */
  void getOutputNames(std::vector<std::string>& name);

  /**
   * describe the context as a string
   */
  std::string describeContext();

  virtual ~EtaslDriver();
};

}  // namespace etasl_ros_controllers
