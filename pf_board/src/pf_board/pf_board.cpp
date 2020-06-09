/// Copyright 2020 Joseph Lee Yuan Sheng
///

#include "pf_board/hardware_interface.h"
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
  std::string subscriber_joint_jog_topic;
  std::string driver_dev_path;

  if (!nodeHandle_.getParam("subscriber_topic", subscriber_joint_jog_topic))
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
  else if (!nodeHandle_.getParam("driver_dev_path", driver_dev_path))
  {
    ROS_ERROR("no param named 'driver_dev_path'");
  }
  else if (!initBoardComms(driver_dev_path))
  {
    ROS_ERROR("Failed to initialize communication to Board");
  }
  else
  {
    state_machine_.setControl(&control_);
    pub_joint_state_ = nodeHandle_.advertise<sensor_msgs::JointState>("joint_state", 1);
    pub_motor_state_ = nodeHandle_.advertise<pf_msgs::MotorStateArray>("motor_state", 1);
    pub_foot_sensor_ = nodeHandle_.advertise<pf_msgs::FootSensorArray>("foot_sensor", 1);
    success = true;
    ROS_INFO("Successfully launched node.");
  }
  return success;
}



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

