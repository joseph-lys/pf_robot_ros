/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_IMP_DATA_FROM_ROS_H
#define FSM_IMP_DATA_FROM_ROS_H

#include "fsm/base_classes.h"

namespace pf_board
{
namespace fsm
{

struct ImpDataFromRos : public DataFromRos
{
  std::shared_ptr<sensor_msgs::JointState> joint_msg{};
};

}  // namespace fsm 
}  // namespace pf_board

#endif  // FSM_IMP_DATA_FROM_ROS_H