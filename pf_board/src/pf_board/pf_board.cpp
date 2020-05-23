/// Copyright 2020 Joseph Lee Yuan Sheng
///

#include "pf_board/pf_board.h"
#include <string>

using pf_board::PfBoard;


PfBoard::PfBoard(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{

}

PfBoard::~PfBoard()
{
}

bool PfBoard::init()
{
  bool success = false;

  if (!nodeHandle_.getParam("subscriber_topic", subscriber_joint_jog_topic_))
  {
    ROS_ERROR("no param named 'subscriber_topic'");
  }
  else if (!nodeHandle_.getParam("joint_names", joint_name_list_))
  {
    ROS_ERROR("no param named 'joint_names'");
  }
  else if (!nodeHandle_.getParam("motor_ids", motor_id_list_))
  {
    ROS_ERROR("no param named 'motor_ids'");
  }
  else if ((joint_name_list_.size() != motor_id_list_.size())
           || (joint_name_list_.size() < 1))
  {
    ROS_ERROR("invalid size for param 'joint_names' and 'motor_ids'");
  }
  else
  {
    pub_joint_state_ = nodeHandle_.advertise<sensor_msgs::JointState>("joint_state", 1);
    pub_motor_state_ = nodeHandle_.advertise<pf_msgs::MotorStateArray>("motor_state", 1);
    pub_foot_sensor_ = nodeHandle_.advertise<pf_msgs::FootSensorArray>("foot_sensor", 1);
    success = true;
  }
  
  return success;
}
//   if (!readParameters())
//   {
//     ROS_ERROR("Could not read parameters.");
//     ros::requestShutdown();
//   }
//   // subscriber_ = nodeHandle_.subscribe(subscriber_topic_, 1,
//   //                                     &RosPackageTemplate::topicCallback, this);
//   // serviceServer_ = nodeHandle_.advertiseService("get_average",
//   //                                               &RosPackageTemplate::serviceCallback, this);
//   ROS_INFO("Successfully launched node.");
// }


// bool PfBoard::readParameters()
// {
//   if (!nodeHandle_.getParam("subscriber_topic", subscriber_topic_)) return false;
//   return true;
// }

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

