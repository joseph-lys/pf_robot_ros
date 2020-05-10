/// Copyright 2020 Joseph Lee Yuan Sheng
///

#include "pf_board/pf_board.h"
#include <string>

using pf_board::PfBoard;


PfBoard::PfBoard(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  // subscriber_ = nodeHandle_.subscribe(subscriber_topic_, 1,
  //                                     &RosPackageTemplate::topicCallback, this);
  // serviceServer_ = nodeHandle_.advertiseService("get_average",
  //                                               &RosPackageTemplate::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

PfBoard::~PfBoard()
{
}

bool PfBoard::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriber_topic_)) return false;
  return true;
}

// void RosPackageTemplate::topicCallback(const sensor_msgs::Temperature& message)
// {
//   algorithm_.addData(message.temperature);
// }

// bool RosPackageTemplate::serviceCallback(std_srvs::Trigger::Request& request,
//                                          std_srvs::Trigger::Response& response)
// {
//   response.success = true;
//   response.message = "The average is " + std::to_string(algorithm_.getAverage());
//   return true;
// }

