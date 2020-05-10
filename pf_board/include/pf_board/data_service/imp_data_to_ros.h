/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_IMP_DATA_TO_ROS_H
#define FSM_IMP_DATA_TO_ROS_H

#include <memory>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "pf_msgs/MotorStateArray.h"
#include "pf_msgs/InputState.h"
#include "sensor_msgs/JointState.h"
#include "fsm/base_classes.h"


namespace pf_board
{
namespace fsm
{

struct ImpDataToRos : public DataToRos
{
  std::shared_ptr<sensor_msgs::JointState> mag_msg{};
  std::shared_ptr<sensor_msgs::MagneticField> mag_msg{};
  std::shared_ptr<sensor_msgs::Imu> Imu_msg{};
  std::shared_ptr<pf_msgs::MotorStateArray> motor_msg{};
  std::shared_ptr<pf_msgs::InputState> inputs_msg{};
};

}  // namespace fsm 
}  // namespace pf_board

#endif  // FSM_IMP_DATA_TO_ROS_H