/// Copyright 2020 Joseph Lee Yuan Sheng
///
/// loosely based on https://github.com/resibots/dynamixel_control_hw.git

#ifndef PF_BOARD_HARDWARE_INTERFACE_H
#define PF_BOARD_HARDWARE_INTERFACE_H

#include <vector>
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


namespace pf_board
{

class PfBoardHw : public hardware_interface::RobotHW
{
 public:
  init(ros::NodeHandle& controller_nh, ros::NodeHandle& hw_nh) override;
  void read(const ros::Time& /*time*/, const ros::Duration& period) override;
  void write(const ros::Time& /*time*/, const ros::Duration& period) override;
 private: 
  bool initParams();
  bool initBoardInterface();
  bool initRosInterface(ros::NodeHandle& controller_nh, ros::NodeHandle& hw_nh);

  // Mapping data
  std::vector<std::string> joint_names_;  // joint names
  std::vector<>
  std::vector<uint8_t> motor_ids_;  // motor ids configuration
  std::unordered_map<std::string, size_t> name2index_;  // lookup table for 
  std::unordered_map<uint8_t, std::string> id2name_; // 
  
  // 
  std::vector<double> goal_position_;  // motors' target position in rads
  std::vector<double> moving_speed_;  // motors' target moving speed in rads/s

  std::vector<double> current_position_;  // motors' current position in rads
  std::vector<double> current_velocity_;  // motors' current rotational velocity in rads/s
  std::vector<double> present_load_;  // motors' current torque in Nm
  std::vector<double> force_sensors_;  // force on foot sensors in N

  // ros_control interface for joint states and commands
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface pos_cmd_interface_;
  hardware_interface::PosVelJointInterface pos_vel_cmd_interface_;

  // ros_control interface for force sensors
  hardware_interface::ForceTorqueSensorInterface force_sensor_interface_;

  /// Additional state control
  bool torque_enable_;  // enable/disable motor torques. initialized to false (torque disabled) 
  bool control_enable_;  // if false, the control will not send any data. initialized to false (send configuration only)
};


}  // namespace pf_board


#endif   // PF_BOARD_HARDWARE_INTERFACE_H



using pf_board::PfBoardHw;

bool PFBoardHw::init(ros::NodeHandle& controller_nh, ros::NodeHandle& hw_nh)
{
  bool success = false;
  if(!initParams(hw_nh))
  {
    char error_message[] = "Failed to load params"
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }
  else if (!initBoardInterface(hw_nh))
  {
    char error_message[] = "Failed to initialize communication to PF board"
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }
  else if (!initRosInterface(controller_nh, hw_nh))
  {
    char error_message[] = "Failed to register Hardware Controller";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }
  else
  {
    success = true;
  }
  return success;
}

bool PFBoard::initParams()
{
  ros::NodeHandle nh("~");
  bool success_controller_params = false;
  bool success_joint_params = false;
  bool success_sensor_params = false;
  /// get loop rate

  if (!success_controller_params)
  {
    // previous step has error, dont bother continuing
  }
  else if(!nh.getParam("joint_names", joint_names_)
  {
    char error_message[] = "Missing parameters 'joint_names' parameter";
    ROS_ERROR_STREAM(error_message
      << "(" << nh.getNamespace() << '/' << "joint_names)");
    throw std::runtime_error(error_message);
  }
  else if(!nh.getParam("motor_ids", motor_ids_)
  {
    char error_message[] = "Missing parameters 'motor_ids' parameter ";
    ROS_ERROR_STREAM(error_message
      << "(" << nh.getNamespace() << '/' << "motor_ids)");
    throw std::runtime_error(error_message);
  }
  else if (joint_names_.size() != motor_ids_.size())
  {
    char error_message[] = "Mismatch size for paramaters 'joint_names' and 'motor_ids'";
    ROS_ERROR(error_message
      << "(joint_names size [" << joint_names_.size() << "] !=" 
      << "(motor_ids size [" << motor_ids_.size() << "])");
    throw std::runtime_error(error_message);
  }
  else if (joint_names_.size() < 1)
  {
    char error_message[] = "Expected at least one item in 'joint_names' and 'motor_ids";
    ROS_ERROR(error_message);
    throw std::runtime_error(error_message);
  }
  else if (joint_names_.size() > 32)
  {
    ROS_ERROR(error_message
      << " (" <<joint_names_.size() << " > 32)");
    throw std::runtime_error(error_message);
  }
  else
  {
    success_joint_params = true;
    for (int i=0; i<joint_names_.size(), i++)
    {
      if (motor_ids_[i] >= 32)
      {
        success_joint_params = false;
        char error_message[] = "Only supports 'motor_ids' values up to 31";
        ROS_ERROR_STREAM(error_message 
          "(configuration for joint '"<< joint_names_[i] << "' is " << motor_ids_[i] << ")");
        throw std::runtime_error(error_message);
      }
      else
      {
        name2index_[joint_names_[i]] = i;
        id2name_[motor_ids_[i]] = joint_names_[i];
      }
    }
  }

  if (!success_joint_params)
  {
    // previous step has error, dont bother continuing
  }
  else if(!nh.getParam("sensor_names", sensor_names_)
  {
    char error_message[] = "Missing parameters 'sensor_names' parameter";
    ROS_ERROR_STREAM(error_message
      << "(" << nh.getNamespace() << '/' << "sensor_names");
    throw std::runtime_error(error_message);
  }
  else if(!nh.getParam("frame_ids", frame_ids_)
  {
    char error_message[] = "Missing parameters 'frame_ids' parameter ";
    ROS_ERROR_STREAM(error_message
      << "(" << nh.getNamespace() << '/' << "frame_ids)");
    throw std::runtime_error(error_message);
  }
  else if (sensor_names_.size() != frame_ids_.size())
  {
    char error_message[] = "Mismatch size for paramaters 'joint_names' and 'motor_ids'";
    ROS_ERROR(error_message
      << "(joint_names size [" << sensor_names_.size() << "] !=" 
      << "(frame_ids_ size [" << frame_ids_.size() << "])");
    throw std::runtime_error(error_message);
  }

  return success_controller_params && success_joint_params && success_sensor_params;
}

bool PFBoard::initBoardInterface()
{
  /// configure the data based on the received params
  goal_position_.resize(joint_names_.size(), 0.0);
  moving_speed_.resize(joint_names_.size(), 0.0);

  current_position_.resize(joint_names_.size(), 0.0);
  current_velocity_.resize(joint_names_.size(), 0.0);
  present_load_.resize(joint_names_.size(), 0.0);

  force_sensors_.resize(sensor_names_.size(), 0.0);
}

bool PFBoard::initRosInterface(ros::NodeHandle& controller_nh, ros::NodeHandle& hw_nh)
{
  for (size_t i=0; i < joint_names_.size(), i++)
  {
    // Handle for joint state feedback
    auto jnt_state_handle = hardware_interface::JointStateHandle(
      joint_names_[i],         // name
      &current_position_[i],   // pos
      &current_velocitiy_[i],  // vel
      &present_load_[i],       // eff
    )
    jnt_state_interface_.registerHandle(jnt_state_handle);

    // Handle for position only control
    auto pos_cmd_handle = hardware_interface::JointHandle(
      joint_names_[i],     // name
      &goal_position_[i]   // cmd
    )
    pos_cmd_interface_.registerHandle(pos_cmd_handle);

    // Handle for position + velocity control
    auto pos_vel_cmd_handle = hardware_interface::PosVelJointHandle(
      joint_names[i],
      &goal_position_[i]   // cmd_pos
      &moving_speed_[i]    // cmd_vel
    )
    pos_vel_cmd_interface_.registerHandle(pos_vel_cmd_handle);

    // jnt_state_handle
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&pos_cmd_interface_);
  registerInterface(&pos_vel_cmd_interface_);

  for (size_t i=0; i < sensor_names_.size(), i++)
  {
    // Handle for joint state feedback
    auto force_sensor_handle = hardware_interface::ForceTorqueSensorHandle(
      sensor_names_[i],         // name
      frame_ids_[i],            // frame_id
      &force_sensors_[i],       // force
      nullptr                   // torque
    )
    force_sensor_interface_.registerHandle(force_sensor_handle);
  }
  registerInterface(&force_sensor_interface_);
}

bool PFBoard::hwTransferOnce(bool send_command, bool motor_enable)
{

}

bool PFBoard::setToInitialValues()
{
  /// initilialize command values after startup. This is called after first successful feedback
  for (size_t i=0; i < joint_names_.size(); i++)
  {
    goal_position_[i] = current_position_[i];
    moving_speed_[i] = 0.0  // WARNING: zero means not used, will use MAXIMUM SPEED!
  }
}

  