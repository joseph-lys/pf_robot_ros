/// Copyright 2020 Joseph Lee Yuan Sheng
///
/// loosely based on https://github.com/resibots/dynamixel_control_hw.git

#ifndef PF_BOARD_HARDWARE_INTERFACE_H
#define PF_BOARD_HARDWARE_INTERFACE_H

#include <vector>
#include <unordered_map>
#include <ros/ros.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <pf_msgs/MotorStates.h>
#include <pf_board/time_source.h>
#include <pf_board/simple_timer.h>
#include <pf_board/comms/transport_layer.h>
#include <pf_board/comms/spi_driver.h>
#include <pf_board/dxl_convert.h>
#include "command_struct.h"
#include "feedback_struct.h"


namespace pf_board
{


class PfBoardHw : public hardware_interface::RobotHW
{
 public:
  bool init(ros::NodeHandle& controller_nh, ros::NodeHandle& hw_nh) override;
  void read(const ros::Time& /*time*/, const ros::Duration& period) override;
  void write(const ros::Time& /*time*/, const ros::Duration& period) override;
 private: 
  bool initParams();
  bool initBoardInterface();
  bool initRosInterface(ros::NodeHandle& controller_nh, ros::NodeHandle& hw_nh);
  pf_msgs::MotorStates createMotorStateMsg();

  // Mapping data
  std::vector<pf_board::DxlConvert> motors_;
  std::unordered_map<std::string, std::size_t> name2index_;  // lookup table for 
  std::unordered_map<uint8_t, std::size_t> id2index_;  // lookup table for id 
  
  std::vector<double> goal_position_;  // motors' target position in rads
  std::vector<double> moving_speed_;  // motors' target moving speed in rads/s

  std::vector<double> present_position_;  // motors' current position in rads
  std::vector<double> present_velocity_;  // motors' current rotational velocity in rads/s
  std::vector<double> present_load_;  // motors' current torque in Nm
  std::vector<double> force_sensors_;  // force on foot sensors in N

  // ros_control interface for joint states and commands
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface pos_cmd_interface_;
  hardware_interface::PosVelJointInterface pos_vel_cmd_interface_;

  // ros_control interface for force sensors
  hardware_interface::ForceTorqueSensorInterface force_sensor_interface_;

  /// Additional state control
  std::vector<uint8_t> status_;  // motor's status byte
  bool comm_ok_;
  bool torque_enable_;  // enable/disable motor torques. initialized to false (torque disabled) 
  bool control_enable_;  // if false, the control will not send any data. initialized to false (send configuration only)

  /// Transfer buffer and Structs
  uint8_t tx_buffer_[256];
  uint8_t rx_buffer_[256];

  /// stuff for msg header
  uint32_t seq_ = 0;

  pf_board::comms::SpiDriver* p_driver_ = nullptr;
};


}  // namespace pf_board


#endif   // PF_BOARD_HARDWARE_INTERFACE_H
