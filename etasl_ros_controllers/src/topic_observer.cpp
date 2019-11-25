// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the LGPL-3.0 license, see LICENSE

/*
 * Author: Mathias Hauan Arbo
 *
 * This file has been adapted from:
 * https://gitlab.mech.kuleuven.be/rob-expressiongraphs/etasl_rtt/blob/master/src/port_observer.hpp
 */
#include "etasl_ros_controllers/topic_observer.h"

namespace KDL
{

/**
 * Observer that publishes events to a ROS topic
 */

class TopicObserver : public Observer
{
  Context::Ptr ctx;
  const std::string portname;
  Observer::Ptr next;                 ///< the next observer you want to react to monitors.
  boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::String>> outp;
  const std::string action_name;
  const std::string event_postfix;
  bool exit_when_triggered;
public:
  typedef boost::shared_ptr<TopicObserver> Ptr;
  TopicObserver(Context::Ptr _ctx,
		boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::String>> _outp,
		const std::string& _action_name,
		const std::string& _event_postfix,
		bool _exit_when_triggered,
		Observer::Ptr _next)
    : ctx(_ctx), outp(_outp),
      action_name(_action_name),
      event_postfix(_event_postfix),
      exit_when_triggered(_exit_when_triggered),
      next(_next)
  {}
  
  /**
   * The solver will call this when MonitoringScalar is activated.
   * \param [in] mon the monitor that was activated.
   */
  virtual void monitor_activated(const  MonitorScalar& mon)
  {
    std::string ename = mon.argument;
    if (mon.action_name.compare(action_name)==0)
    {
      if (ename.size()==0)
      {
      	ename = "e_finished";
      }
      
      if (event_postfix.size()!=0)
      {
	ename = ename + "@" + event_postfix;
      }
      if (outp->trylock())
      {
	outp->msg_.data = ename;
	outp->unlockAndPublish();
      }
      if (exit_when_triggered)
      {
	ctx->setFinishStatus();
      }
    }
    else
    {
      if (next)
      {
	next->monitor_activated(mon);
      }
    }
  }
  virtual ~TopicObserver() {}
};

Observer::Ptr create_topic_observer(
    Context::Ptr _ctx,
    boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::String>> _outp,
    const std::string& _action_name,
    const std::string& _event_postfix,
    bool  _exit_when_triggered,
    Observer::Ptr _next)
{
  TopicObserver::Ptr r( new TopicObserver(_ctx, _outp, _action_name, _event_postfix, _exit_when_triggered, _next) );
  return r;
}

}
