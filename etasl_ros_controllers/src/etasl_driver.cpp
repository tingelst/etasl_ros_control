#include <iostream>

#include "etasl_ros_controllers/etasl_driver.h"

namespace etasl_ros_controllers
{
bool EtaslDriver::initializeFeatureVariables(double initialization_time, double sample_time, double convergence_crit, DoubleMap& result)
{
  // initialization:
  if (slvr->getNrOfFeatureStates() > 0)
  {
    double t;
    for (t = 0; t < initialization_time; t += sample_time)
    {
      int retval = slvr->updateStep(sample_time);
      if (retval != 0)
      {
        ROS_ERROR_STREAM("initialize_feature_variables: solver encountered the following error during initialization (t=" << t << " )");
        ROS_ERROR_STREAM(slvr->errorMessage(retval));
        ROS_ERROR_STREAM(ctx);
        return false;
      }
      double norm_change = slvr->getNormChange() * sample_time;
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
  // slvr->setInitialValues(); // sets the initial value fields of variables in the context.
}

EtaslDriver::EtaslDriver(int nWSR, double cputime, double regularization_factor)
{
  /*ROS_INFO_STREAM( "constructor EtaslDriver" );*/

  ctx = boost::make_shared<Context>();
  ctx->addType("robot");
  ctx->addType("feature");
  time_ndx = ctx->getScalarNdx("time");
  lua = boost::make_shared<LuaContext>();
  lua->initContext(ctx);

  slvr = boost::make_shared<qpOASESSolver>(nWSR, cputime, regularization_factor);

  //   obs = boost::make_shared<PythonObserver>(ctx);
  //   ctx->addDefaultObserver(obs);

  etaslread = false;
  initialized = false;
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

int EtaslDriver::setInput(const DoubleMap& dmap)
{
  for (DoubleMap::const_iterator it = dmap.begin(); it != dmap.end(); ++it)
  {
    // ROS_INFO_STREAM( it->first << "\t=\t" << it->second );
    VariableType<double>::Ptr v = ctx->getInputChannel<double>(it->first);
    if (v)
    {
      v->setValue(it->second);
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
    VariableType<double>::Ptr v = ctx->getInputChannel<double>(it->first);
    if (v)
    {
      v->setJacobian(time_ndx, it->second);
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
  if (!initialized)
    return -1;
  int count = 0;
  for (unsigned int i = 0; i < jnames.size(); ++i)
  {
    DoubleMap::const_iterator e = dmap.find(jnames[i]);
    if (e != dmap.end())
    {
      jvalues[i] = e->second;
      ++count;
    }
  }
  for (unsigned int i = 0; i < fnames.size(); ++i)
  {
    DoubleMap::const_iterator e = dmap.find(fnames[i]);
    if (e != dmap.end())
    {
      fvalues[i] = e->second;
      ++count;
    }
  }
  slvr->setJointStates(jvalues);
  slvr->setFeatureStates(fvalues);
  DoubleMap::const_iterator e = dmap.find("time");
  if (e != dmap.end())
  {
    slvr->setTime(e->second);
    ++count;
  }
  return count;
}

int EtaslDriver::getJointVel(DoubleMap& dmap, int flag)
{
  if (!initialized)
    return -1;
  slvr->getJointVelocities(jvelocities);
  slvr->getFeatureVelocities(fvelocities);
  dmap.clear();
  if ((flag == 1) || (flag == 3))
  {
    for (unsigned int i = 0; i < jnames.size(); ++i)
    {
      dmap[jnames[i]] = jvelocities[i];
    }
    dmap["time"] = 1.0;
  }
  if ((flag == 2) || (flag == 3))
  {
    for (unsigned int i = 0; i < fnames.size(); ++i)
    {
      dmap[fnames[i]] = fvelocities[i];
    }
  }
  return 0;
}

int EtaslDriver::getJointPos(DoubleMap& dmap, int flag)
{
  if (!initialized)
    return -1;
  slvr->getJointStates(jvalues);
  slvr->getFeatureStates(fvalues);
  dmap.clear();
  if ((flag == 1) || (flag == 3))
  {
    for (unsigned int i = 0; i < jnames.size(); ++i)
    {
      dmap[jnames[i]] = jvalues[i];
    }
    dmap["time"] = slvr->getTime();
  }
  if ((flag == 2) || (flag == 3))
  {
    for (unsigned int i = 0; i < fnames.size(); ++i)
    {
      dmap[fnames[i]] = fvalues[i];
    }
  }
  return 0;
}

void EtaslDriver::getOutput(DoubleMap& dmap)
{
  for (Context::OutputVarMap::iterator it = ctx->output_vars.begin(); it != ctx->output_vars.end(); it++)
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
  int retval = slvr->prepareInitialization(ctx);
  if (retval != 0)
  {
    ROS_INFO_STREAM(" : etasl_rtt::initialize() - prepareInitialization : the taskspecification contains priority levels that the numerical solver "
                    "can't handle. ");
    return -1;
  }

  slvr->getJointNameVector(jnames);
  jvalues.setZero(jnames.size());
  jvelocities.setZero(jnames.size());

  slvr->getFeatureNameVector(fnames);
  fvalues.setZero(fnames.size());
  fvelocities.setZero(fnames.size());

  slvr->getJointStates(jvalues);
  slvr->getFeatureStates(fvalues);
  slvr->setTime(0.0);
  initialized = true;

  setJointPos(initialval);

  // performs optimization and writes the result in the "initial" field in the context.
  if (!initializeFeatureVariables(initialization_time, sample_time, convergence_crit, convergedval))
  {
    initialized = false;
    return -2;
  }

  //********************* Preparing for normal execution  ************************
  // initializes variables from their "initial" field (including feature variables)
  retval = slvr->prepareExecution(ctx);
  if (retval != 0)
  {
    ROS_INFO_STREAM(" : etasl_rtt::initialize() - prepareExecution : the taskspecification contains priority levels that the numerical solver can't "
                    "handle. ");
    initialized = false;
    return -3;
  }

  slvr->getJointNameVector(jnames);
  jvalues.setZero(jnames.size());
  jvelocities.setZero(jnames.size());

  slvr->getFeatureNameVector(fnames);
  fvalues.setZero(fnames.size());
  fvelocities.setZero(jnames.size());

  slvr->getJointStates(jvalues);
  slvr->getFeatureStates(fvalues);
  slvr->setTime(0.0);
  ctx->resetMonitors();
  ctx->clearFinishStatus();
  initialized = true;

  setJointPos(convergedval);

  retval = slvr->solve();
  if (retval != 0)
  {
    ROS_INFO_STREAM("solver encountered the following error during the first solve in initialize \nmessage: " << slvr->errorMessage(retval).c_str()
                                                                                                              << "\n stop() will be called on etasl "
                                                                                                                 "rtt component and e_error event "
                                                                                                                 "will be send"
                                                                                                              << "\n");
    initialized = false;
    return -4;
  }
  return 0;
}

int EtaslDriver::solve()
{
  int retval = slvr->solve();
  if (retval != 0)
  {
    ROS_INFO_STREAM("solved encountered error during computations in updateHook()"
                    << " ) \nmessage: " << slvr->errorMessage(retval).c_str()
                    << "\n stop() will be called on etasl rtt component and e_error event will be sent"
                    << "\n"
                    << ctx);
    return -1;
  }
  ctx->checkMonitors();
  if (ctx->getFinishStatus())
  {
    return 1;
  }
  return 0;
}

void EtaslDriver::evaluate()
{
  slvr->evaluate_expressions();
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
  if (initialized)
  {
    return fvalues.size();
  }
  else
  {
    return -1;
  }
}

int EtaslDriver::nrOfRobotVar()
{
  if (initialized)
  {
    return jvalues.size();
  }
  else
  {
    return -1;
  }
}

int EtaslDriver::nrOfScalarConstraints()
{
  return ctx->cnstr_scalar.size();
}

int EtaslDriver::nrOfBoxConstraints()
{
  return ctx->cnstr_box.size();
}

/**
 * request all declared input channels
 */
void EtaslDriver::getInputNames(std::vector<std::string>& name)
{
  name.resize(ctx->input_vars.size());
  int count = 0;
  for (Context::InputVarMap::iterator it = ctx->input_vars.begin(); it != ctx->input_vars.end(); ++it)
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
  name.resize(ctx->output_vars.size());
  int count = 0;
  for (Context::OutputVarMap::iterator it = ctx->output_vars.begin(); it != ctx->output_vars.end(); ++it)
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
    ctx->getScalarsOfType("robot", all_ndx);
  }
  if ((flag == 2) || (flag == 3))
  {
    ctx->getScalarsOfType("feature", all_ndx);
  }
  name.resize(all_ndx.size());
  weight.resize(all_ndx.size());
  initval.resize(all_ndx.size());
  for (size_t i = 0; i < all_ndx.size(); ++i)
  {
    VariableScalar* vs = ctx->getScalarStruct(all_ndx[i]);
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