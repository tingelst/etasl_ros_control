#include <iostream>

#include "etasl_ros_controllers/etasl_driver.h"

namespace etasl_ros_controllers
{
bool EtaslDriver::initializeFeatureVariables(double initialization_time, double sample_time, double convergence_crit, DoubleMap& result)
{
  // initialization:
  if (solver_->getNrOfFeatureStates() > 0)
  {
    double t;
    for (t = 0; t < initialization_time; t += sample_time)
    {
      int retval = solver_->updateStep(sample_time);
      if (retval != 0)
      {
        ROS_ERROR_STREAM("initialize_feature_variables: solver encountered the following error during initialization (t=" << t << " )");
        ROS_ERROR_STREAM(solver_->errorMessage(retval));
        ROS_ERROR_STREAM(ctx_);
        return false;
      }
      double norm_change = solver_->getNormChange() * sample_time;
      if (norm_change <= convergence_crit)
        break;
    }
  }
  int nr = getJointPos(result, 3);

  if (nr < 0)
  {
    return false;
  }
  else
  {
    return true;
  }
  // solver_->setInitialValues(); // sets the initial value fields of variables in the context.
}

EtaslDriver::EtaslDriver(int nWSR, double cputime, double regularization_factor)
{
  /*ROS_INFO_STREAM( "constructor EtaslDriver" );*/

  ctx_ = boost::make_shared<Context>();
  ctx_->addType("robot");
  ctx_->addType("feature");
  time_ndx_ = ctx_->getScalarNdx("time");
  lua = boost::make_shared<LuaContext>();
  lua->initContext(ctx_);

  solver_ = boost::make_shared<qpOASESSolver>(nWSR, cputime, regularization_factor);

  obs_ = create_default_observer(ctx_, "exit");
  ctx_->addDefaultObserver(obs_);

  etaslread = false;
  initialized_ = false;
  all_ndx.reserve(300);
}

void EtaslDriver::readTaskSpecificationFile(const std::string& filename)
{
  lua->executeFile(filename);
  etaslread = true;
}

void EtaslDriver::readTaskSpecificationString(const std::string& taskspec)
{
  lua->executeString(taskspec);
  etaslread = true;
}

// int EtaslDriver::setInput(const DoubleMap& dmap)
// {
//   for (DoubleMap::const_iterator it = dmap.begin(); it != dmap.end(); ++it)
//   {
//     // ROS_INFO_STREAM( it->first << "\t=\t" << it->second );
//     VariableType<double>::Ptr v = ctx_->getInputChannel<double>(it->first);
//     if (v)
//     {
//       v->setValue(it->second);
//     }
//     else
//     {
//       return -1;
//     }
//   }
//   return 0;
// }

int EtaslDriver::setInput(const DoubleMap& dmap)
{
  for (auto item : dmap)
  {
    auto v = ctx_->getInputChannel<double>(item.first);
    if (v)
    {
      v->setValue(item.second);
    }
    else
    {
      return -1;
    }
  }
  return 0;
}

int EtaslDriver::setInput(const FrameMap& fmap)
{
  for (auto item : fmap)
  {
    auto v = ctx_->getInputChannel<Frame>(item.first);
    if (v)
    {
      v->setValue(item.second);
    }
    else
    {
      return -1;
    }
  }
  return 0;
}

int EtaslDriver::setInputVelocity(const DoubleMap& dmap)
{
  for (DoubleMap::const_iterator it = dmap.begin(); it != dmap.end(); ++it)
  {
    VariableType<double>::Ptr v = ctx_->getInputChannel<double>(it->first);
    if (v)
    {
      v->setJacobian(time_ndx_, it->second);
    }
    else
    {
      return -1;
    }
  }
  return 0;
}

int EtaslDriver::setJointPos(const DoubleMap& dmap)
{
  if (!initialized_)
  {
    return -1;
  }

  int count = 0;
  for (unsigned int i = 0; i < joint_names_.size(); ++i)
  {
    DoubleMap::const_iterator e = dmap.find(joint_names_[i]);
    if (e != dmap.end())
    {
      joint_values_[i] = e->second;
      ++count;
    }
  }
  for (unsigned int i = 0; i < feature_names_.size(); ++i)
  {
    DoubleMap::const_iterator e = dmap.find(feature_names_[i]);
    if (e != dmap.end())
    {
      feature_values_[i] = e->second;
      ++count;
    }
  }
  solver_->setJointStates(joint_values_);
  solver_->setFeatureStates(feature_values_);
  DoubleMap::const_iterator e = dmap.find("time");
  if (e != dmap.end())
  {
    solver_->setTime(e->second);
    ++count;
  }
  return count;
}

int EtaslDriver::setJointPos(const std::vector<double>& joint_positions, std::vector<std::string>& joint_names)
{
  if (!initialized_)
  {
    return -1;
  }

  DoubleMap position_map;
  std::transform(joint_names.begin(), joint_names.end(), joint_positions.begin(), std::inserter(position_map, position_map.end()),
                 [](std::string a, double b) { return std::make_pair(a, b); });

  return setJointPos(position_map);
}

int EtaslDriver::getJointVel(DoubleMap& dmap, int flag)
{
  if (!initialized_)
  {
    return -1;
  }
  solver_->getJointVelocities(joint_velocities_);
  solver_->getFeatureVelocities(feature_velocities_);
  dmap.clear();
  if ((flag == 1) || (flag == 3))
  {
    for (unsigned int i = 0; i < joint_names_.size(); ++i)
    {
      dmap[joint_names_[i]] = joint_velocities_[i];
    }
    dmap["time"] = 1.0;
  }
  if ((flag == 2) || (flag == 3))
  {
    for (unsigned int i = 0; i < feature_names_.size(); ++i)
    {
      dmap[feature_names_[i]] = feature_velocities_[i];
    }
  }
  return 0;
}

int EtaslDriver::getJointPos(DoubleMap& dmap, int flag)
{
  if (!initialized_)
    return -1;
  solver_->getJointStates(joint_values_);
  solver_->getFeatureStates(feature_values_);
  dmap.clear();
  if ((flag == 1) || (flag == 3))
  {
    for (unsigned int i = 0; i < joint_names_.size(); ++i)
    {
      dmap[joint_names_[i]] = joint_values_[i];
    }
    dmap["time"] = solver_->getTime();
  }
  if ((flag == 2) || (flag == 3))
  {
    for (unsigned int i = 0; i < feature_names_.size(); ++i)
    {
      dmap[feature_names_[i]] = feature_values_[i];
    }
  }
  return 0;
}

void EtaslDriver::getOutput(DoubleMap& dmap)
{
  for (Context::OutputVarMap::iterator it = ctx_->output_vars.begin(); it != ctx_->output_vars.end(); it++)
  {
    Expression<double>::Ptr expr = boost::dynamic_pointer_cast<Expression<double> >(it->second);
    if (expr)
    {
      dmap[it->first] = expr->value();
    }
  }
}

int EtaslDriver::initialize(const DoubleMap& initialval, double initialization_time, double sample_time, double convergence_crit,
                            DoubleMap& convergedval)
{
  if (!etaslread)
    return -5;

  //********************* Initialization optimisation ************************

  // prepares matrices etc and
  // initializes variables from their "initial" field (including feature variables)
  int retval = solver_->prepareInitialization(ctx_);
  if (retval != 0)
  {
    ROS_INFO_STREAM(" : etasl_rtt::initialize() - prepareInitialization : the taskspecification contains priority "
                    "levels that the numerical solver "
                    "can't handle. ");
    return -1;
  }

  solver_->getJointNameVector(joint_names_);
  joint_values_.setZero(joint_names_.size());
  joint_velocities_.setZero(joint_names_.size());

  solver_->getFeatureNameVector(feature_names_);
  feature_values_.setZero(feature_names_.size());
  feature_velocities_.setZero(feature_names_.size());

  solver_->getJointStates(joint_values_);
  solver_->getFeatureStates(feature_values_);
  solver_->setTime(0.0);
  initialized_ = true;

  setJointPos(initialval);

  // performs optimization and writes the result in the "initial" field in the context.
  if (!initializeFeatureVariables(initialization_time, sample_time, convergence_crit, convergedval))
  {
    initialized_ = false;
    return -2;
  }

  //********************* Preparing for normal execution  ************************
  // initializes variables from their "initial" field (including feature variables)
  retval = solver_->prepareExecution(ctx_);
  if (retval != 0)
  {
    ROS_INFO_STREAM(" : etasl_rtt::initialize() - prepareExecution : the taskspecification contains priority levels "
                    "that the numerical solver can't "
                    "handle. ");
    initialized_ = false;
    return -3;
  }

  solver_->getJointNameVector(joint_names_);
  joint_values_.setZero(joint_names_.size());
  joint_velocities_.setZero(joint_names_.size());

  solver_->getFeatureNameVector(feature_names_);
  feature_values_.setZero(feature_names_.size());
  feature_velocities_.setZero(joint_names_.size());

  solver_->getJointStates(joint_values_);
  solver_->getFeatureStates(feature_values_);
  solver_->setTime(0.0);
  ctx_->resetMonitors();
  ctx_->clearFinishStatus();
  initialized_ = true;

  setJointPos(convergedval);

  retval = solver_->solve();
  if (retval != 0)
  {
    ROS_INFO_STREAM("solver encountered the following error during the first solve in initialize \nmessage: " << solver_->errorMessage(retval).c_str()
                                                                                                              << "\n stop() will be called on etasl "
                                                                                                                 "rtt component and e_error event "
                                                                                                                 "will be send"
                                                                                                              << "\n");
    initialized_ = false;
    return -4;
  }
  return 0;
}

int EtaslDriver::solve()
{
  int retval = solver_->solve();
  if (retval != 0)
  {
    ROS_INFO_STREAM("solved encountered error during computations in updateHook()"
                    << " ) \nmessage: " << solver_->errorMessage(retval).c_str()
                    << "\n stop() will be called on etasl rtt component and e_error event will be sent"
                    << "\n"
                    << ctx_);
    return -1;
  }
  ctx_->checkMonitors();
  if (ctx_->getFinishStatus())
  {
    return 1;
  }
  return 0;
}

void EtaslDriver::evaluate()
{
  solver_->evaluate_expressions();
}

// std::string EtaslDriver::getEvent()
// {
//   return obs->action_name;
// }

EtaslDriver::~EtaslDriver()
{
}

int EtaslDriver::nrOfFeatureVar()
{
  if (initialized_)
  {
    return feature_values_.size();
  }
  else
  {
    return -1;
  }
}

int EtaslDriver::nrOfRobotVar()
{
  if (initialized_)
  {
    return joint_values_.size();
  }
  else
  {
    return -1;
  }
}

int EtaslDriver::nrOfScalarConstraints()
{
  return ctx_->cnstr_scalar.size();
}

int EtaslDriver::nrOfBoxConstraints()
{
  return ctx_->cnstr_box.size();
}

/**
 * request all declared input channels
 */
void EtaslDriver::getInputNames(std::vector<std::string>& name)
{
  name.resize(ctx_->input_vars.size());
  int count = 0;
  for (Context::InputVarMap::iterator it = ctx_->input_vars.begin(); it != ctx_->input_vars.end(); ++it)
  {
    name[count] = it->first;
    count++;
  }
}

/**
 * request all declared output expressions
 */
void EtaslDriver::getOutputNames(std::vector<std::string>& name)
{
  name.resize(ctx_->output_vars.size());
  int count = 0;
  for (Context::OutputVarMap::iterator it = ctx_->output_vars.begin(); it != ctx_->output_vars.end(); ++it)
  {
    name[count] = it->first;
    count++;
  }
}

void EtaslDriver::getVariables(int flag, std::vector<std::string>& name, std::vector<double>& weight, std::vector<double>& initval)
{
  all_ndx.clear();
  if ((flag == 1) || (flag == 3))
  {
    ctx_->getScalarsOfType("robot", all_ndx);
  }
  if ((flag == 2) || (flag == 3))
  {
    ctx_->getScalarsOfType("feature", all_ndx);
  }
  name.resize(all_ndx.size());
  weight.resize(all_ndx.size());
  initval.resize(all_ndx.size());
  for (size_t i = 0; i < all_ndx.size(); ++i)
  {
    VariableScalar* vs = ctx_->getScalarStruct(all_ndx[i]);
    name[i] = vs->name;
    weight[i] = vs->weight->value();
    initval[i] = vs->initial_value;
  }
}

// std::string EtaslDriver::describeContext()
// {
//   return lua->describeContext();
// }

}  // namespace etasl_ros_controllers