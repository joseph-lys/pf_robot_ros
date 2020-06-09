#include "pf_board/hardware_interface.h"


using pf_board::PfBoardHw;
using pf_board::comms::TransportLayer;
using pf_board::comms::TypeInfo;

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
  std::vector<std::string> joint_names;
  std::vector<uint8_t> motor_ids;
  std::vector<double> offsets;
  std::vector<bool> inverts;
  
  bool success_controller_params = false;
  bool success_joint_params = false;
  bool success_sensor_params = false;
  /// get loop rate

  if (!success_controller_params)
  {
    // previous step has error, dont bother continuing
  }
  else if(!nh.getParam("joint_names", joint_names)
  {
    char error_message[] = "Missing parameter 'joint_names'";
    ROS_ERROR_STREAM(error_message
      << " (" << nh.getNamespace() << '/' << "joint_names)");
    throw std::runtime_error(error_message);
  }
  else if(!nh.getParam("motor_ids", motor_ids)
  {
    char error_message[] = "Missing parameter 'motor_ids' ";
    ROS_ERROR_STREAM(error_message
      << " (" << nh.getNamespace() << '/' << "motor_ids)");
    throw std::runtime_error(error_message);
  }
  else if(!nh.getParam("offset", offset)
  {
    char error_message[] = "Missing parameter 'offset' ";
    ROS_ERROR_STREAM(error_message
      << " (" << nh.getNamespace() << '/' << "offset)");
    throw std::runtime_error(error_message);
  }
  else if(!nh.getParam("invert", invert)
  {
    char error_message[] = "Missing parameter 'invert' ";
    ROS_ERROR_STREAM(error_message
      << " (" << nh.getNamespace() << '/' << "invert)");
    throw std::runtime_error(error_message);
  }
  else if (joint_names.size() < 1)
  {
    char error_message[] = "Expected at least one item in 'joint_names'";
    ROS_ERROR(error_message);
    throw std::runtime_error(error_message);
  }
  else if (joint_names_.size() > 32)
  {
    char error_message[] = "Expected at no more than 32 items in 'joint_names'";
    ROS_ERROR(error_message
      << " (" <<joint_names.size() << " > 32)");
    throw std::runtime_error(error_message);
  }
  else if (joint_names.size() != motor_ids.size())
  {
    char error_message[] = "Mismatch size for paramaters 'joint_names' and 'motor_ids'";
    ROS_ERROR(error_message
      << " (joint_names size [" << joint_names.size() << "] !=" 
      << " (motor_ids size [" << motor_ids.size() << "])");
    throw std::runtime_error(error_message);
  }
  else if (joint_names.size() != offsets.size())
  {
    char error_message[] = "Mismatch size for paramaters 'joint_names' and 'offsets'";
    ROS_ERROR(error_message
      << " (joint_names size [" << joint_names.size() << "] !=" 
      << " (offsets size [" << offsets.size() << "])");
    throw std::runtime_error(error_message);
  }
  else if (joint_names.size() != inverts.size())
  {
    char error_message[] = "Mismatch size for paramaters 'joint_names' and 'inverts'";
    ROS_ERROR(error_message
      << " (joint_names size [" << joint_names.size() << "] !=" 
      << " (inverts size [" << inverts.size() << "])");
    throw std::runtime_error(error_message);
  }
  else
  {
    success_joint_params = true;
    for (int i=0; i<joint_names.size(), i++)
    {
      if (motor_ids[i] >= 32)
      {
        success_joint_params = false;
        char error_message[] = "Only supports 'motor_ids' values up to 31";
        ROS_ERROR_STREAM(error_message 
          "(configuration for joint '"<< joint_names_[i] << "' is " << motor_ids_[i] << ")");
        throw std::runtime_error(error_message);
      }
    }
    if (success_joint_params)
    {
      for (int i=0; i<joint_names[i].size(), i++)
      {
        motors_.push_back(pf_board::DxlConvert
          {
            joint_names[i], // std::string joint_name,
            motor_ids[i],   // uint8_t motor_id,
            offsets[i],      // double offset,
            inverts[i],      // bool invert,
            11.938052,      // double max_velocity, assuming AX-12A, 11.938052 rad/s (114 RPM)
            1.5             // max_load, assuming AX-12A stall torque of 1.5Nm
          }
        )
        name2index_[joint_names[i]] = i;
        id2index_[motor_ids[i]] = i;
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
    char error_message[] = "Mismatch size for paramaters 'sensor_names' and 'motor_ids'";
    ROS_ERROR(error_message
      << "(sensor_names size [" << sensor_names_.size() << "] !=" 
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
  for (size_t i=0; i < motors_.size(), i++)
  {
    // Handle for joint state feedback
    auto jnt_state_handle = hardware_interface::JointStateHandle(
      motors_[i].getJointName(),  // name
      &current_position_[i],      // pos
      &current_velocitiy_[i],     // vel
      &present_load_[i],          // eff
    )
    jnt_state_interface_.registerHandle(jnt_state_handle);

    // Handle for position only control
    auto pos_cmd_handle = hardware_interface::JointHandle(
      motors_[i].getJointName(),  // name
      &goal_position_[i]          // cmd
    )
    pos_cmd_interface_.registerHandle(pos_cmd_handle);

    // Handle for position + velocity control
    auto pos_vel_cmd_handle = hardware_interface::PosVelJointHandle(
      motors_[i].getJointName(),  // name
      &goal_position_[i]          // cmd_pos
      &moving_speed_[i]           // cmd_vel
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

bool PFBoard::hwTransferOnce()
{
  bool success = false;
  size_t i, j;
  size_t received;
  TrasportLayer transport;
  CommandStruct cmd;
  FeedbackStruct fback;
  TypeInfo t;
  /// clear buffers
  for (i=0; i<255; i++)
  {
    tx_buffer_[i] = 0;
    rx_buffer_[i] = 0;
  }
  transport.beginWrite(&tx_buffer_[0]);
  cmd.setBuffer(transport.allocateWrite(110, TransportLayer::TypeEnum::control_type));
  cmd.enabled = torque_enable_;
  cmd.size = motor_ids_.size();
  for(i=0; i<motor_ids_.size(); i++)
  {
    cmd.ids[i] = motor_ids_[i];
  }
  if (command_enable_)
  {
    for(i=0; i<motors_.size(); i++)
    {
      cmd.position[i] = motors_[i].encodeGoalPosition(goal_position_[i]);
      cmd.speed[i] = motors_[i].encodeMovingSpeed(moving_speed_[i]);
    }
  }
  transport.finalizeWrite();

  /// transfer
  p_driver_->transfer(&tx_buffer_[0], &rx_buffer_[0], 200);
  pf_board::TimeSource.setStamp();
  received = transport.beginRead(&rx_buffer_[0], feedback_type | motor_cmd_response_type);

  if (received > 0)
  {
    for (i=0; i<received; i++)
    {
      t = transport.nextRead();
      switch(t.type_enum)
      {
        case TransportLayer::TypeEnum::feedback_type:
          success = true;
          comm_ok_ = true;
          fback.setBuffer(t.data_ptr);
          for(j=0; j<motors_.size(); j++)
          {
            status_[j] = fback.status[j];
            present_position_[j] = motors_[j].decodePresentPosition(fback.position[j]);
            present_velocity_[j] = motors_[j].decodePresentVelocity(fback.speed[j]);
            present_load_[j] = motors_[j].decodePresentLoad(fback.load[j]);
          }
          for(j=0; j<force_sensors_.size(); j++)
          {
            force_sensors_[j] = fback.ain[j];
          }
          break;
        case TransportLayer::TypeEnum::motor_cmd_response_type:
          break;
        default:
          break;
      }
    }
  }
  return success;
}

bool PFBoard::setToInitialValues()
{
  /// initilialize command values after startup. This is called after first successful feedback
  for (size_t i=0; i < motors_.size(); i++)
  {
    goal_position_[i] = motors_[i].decodePresentPosition(present_position_[i]);
    moving_speed_[i] = 0.0;  // WARNING: zero means not used, will use MAXIMUM SPEED!
  }
}

pf_msgs::MotorStates PFBoard::createMotorStateMsg()
{
  pf_msgs::MotorStates msg;
  msg.header.seq = seq_;
  seq_++;
  msg.header.stamp = pf_board::TimeSource.getStamp();
  msg.comm_ok = comm_ok_;
  msg.torque_enabled = torque_enable_;
  msg.torque_enabled = control_enable_;
  for(i=0; i<motors_.size(); i++)
  {
    msg.joint_names.pushback(motors_[i].getJointName());
    msg.motor_ids.pushback(motors_[i].getMotorId());
    msg.motor_states.pushback(status_[i]);
  }
}