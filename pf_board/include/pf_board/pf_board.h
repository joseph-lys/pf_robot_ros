/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_H
#define PF_BOARD_H

#include <string>
#include <unordered_map>
#include "ros/ros.h"
#include "pf_msgs/FootSensorArray.h"
#include "pf_msgs/MotorStateArray.h"
#include "control_msgs/JointJog.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "pf_board/states/state_machine.h"
#include "pf_board/comms/spi_driver.h"
#include "pf_board/pf_board_control.h"


namespace pf_board
{

class PfBoard
{
 public:
  explicit PfBoard(ros::NodeHandle& nodeHandle);
  ~PfBoard();
  bool init();
  void executeOnce();

  /// Service Callbacks
  bool motorTorqueSrvCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  bool resetSrvCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
 private:
  ros::NodeHandle& nodeHandle_;

  ros::Subscriber sub_joint_jog_;
  ros::Publisher pub_joint_state_;
  ros::Publisher pub_motor_state_;
  ros::Publisher pub_foot_sensor_;
  std::vector<std::string> joint_name_list_;
  std::vector<int> motor_id_list_;
  std::unordered_map<int, size_t> motor_id_to_index_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;

  pf_board::PfBoardControl control_;
  pf_board::states::StateMachine state_machine_;

  bool initBoardComms(std::string dev_path);
};


}  // namespace pf_board


#endif  // PF_BOARD_H
