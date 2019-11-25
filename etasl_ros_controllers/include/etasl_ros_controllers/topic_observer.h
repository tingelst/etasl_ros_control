// Copyright (c) 2019 Norwegian University of Science and Technology
// Use of this source code is governed by the LGPL-3.0 license, see LICENSE

/*
 * Author: Mathias Hauan Arbo
 *
 * This file has been adapted from:
 * https://gitlab.mech.kuleuven.be/rob-expressiongraphs/etasl_rtt/blob/master/src/port_observer.hpp
 */
#pragma once

#include <expressiongraph/context.hpp>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/String.h>
#include <string>

namespace KDL {

/**
 * Creates an observer that maps a trigger of a monitor in eTaSL to
 * a string on a ros topic.
 * \param _ctx [in] context, when exit_when_triggered==true, then setFinishStatus() is called 
 *             on _ctx when an event is triggered.
 * \param _outp [in] the ROS publisher to publish the event.
 * \param _action_name [in] should match the  action_name that is specified in the monitor 
 * \param _exit_when_triggered [in] when true, the triggered monitor will also cause the component to 
 *        stop.
 * \param _next  [in] points to the next handler for an observer.
 */
Observer::Ptr create_topic_observer(
    Context::Ptr _ctx,
    boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::String>> _outp,
    const std::string& _action_name,
    const std::string& _event_postfix,
    bool  _exit_when_triggered,
    Observer::Ptr _next = Observer::Ptr()
);

} // namespace KDL

