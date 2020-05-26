#include "pf_board/pf_board_control.h"

#include "command_struct.h"
#include "feedback_struct.h"


using pf_board::PfBoardControl;
using pf_board::comms::SpiDriver;


PfBoardControl::PfBoardControl(SpiDriver* p_driver)
{
  p_driver_ = p_driver
}

void PfBoardControl::rosReceiverCallback(const control_msgs::JointJog::ConstPtr& msg)
{
  joint_jog_ = msg;
}

void PfBoardControl::rosReceiverCallback(const control_msgs::JointJog::ConstPtr& msg)
{

  if (joint_jog_)
  {
    for (std::string joint_name: joint_jog_->joint_names)
    {

    }

  }
  joint_jog_->joint_names
  joint_jog_ = {};  /// release the shared pointer
}


enum InitFlags : size_t {
  kInitDriver = 1ULL,
  kInitMapping = 1ULL << 1
}

void PfBoardControl::setBoardDriver(comms::SpiDriver* p_driver)
{
  if (kInitDriver & init_flags_)
  {
    ROS_ERROR("Attempted to reinitialize board driver!");
  }
  else
  {
    p_driver_ = p_driver;
    init_flags_ |= kInitDriver;
  }
}

void PfBoardControl::setMapping(const std::vector<std::string>& joint_names,
                                const std::vector<uint8_t>& motor_ids)
{
  if (kInitMapping & init_flags_)
  {
    ROS_ERROR("Attempted to reinitialize joint_names<->motor_id mapping!");
  }
  else
  {
    if (joint_name.size() != motor_id.size())
    {
      ROS_ERROR("Mismatched size for 'joint_names' and 'motor_ids'");
    }
    else if (joint_names.size() < 1)
    {
      ROS_ERROR("Expected at least one item in 'joint_names' and 'motor_ids");
    }
    else if (joint_names.size() > 32)
    {
      ROS_ERROR("Only supports up to 32 'motor_ids, but %d configurations given", joint_names_.size());
    }
    else
    {
      joint_names_ = joint_names;
      motor_ids_ = motor_ids;
      for (int i=0; i<joint_names_.size(), i++)
      {
        if (motor_ids_[i] >= 32)
        {
          ROS_ERROR("%s's 'motor_id' of %d is not supported (maximum supported id is 32)",
                    joint_names[i], motor_ids[i]);
        }
        else
        {
          joint_names_2_index_[joint_names_[i]] = i;
          motor_id_2_index_[motor_ids_[i]] = i;
        }
      }
      init_flag_ |= kInitMapping;
    }
  }
}

bool PfBoardControl::isInitialized()
{
  return init_flags_ == kInitDriver | kInitMapping;
}


bool PfBoardControl::processDataFromRos()
{
  has_control_data_ = false;
  if(!isInitialized())
  {
    ROS_ERROR("processDataFromRos called when not initialized!");
  }
  else if (joint_jog_)
  {
    has_control_data_ = true;
  }
  return has_control_data_;
}

bool PfBoardControl::transferBoard(bool send_position, bool torque_enable)
{
  int i;
  uint8_t index;
  string joint_name;
  double x;
  has_feedback_data_ = false;
  if(!isInitialized())
  {
    ROS_ERROR("transferBoard called when not initialized!");
  }
  else
  {
    CommandStruct command;
    /// Prepare data
    tansport.writeTo(&buffer_[0]);
    auto type_info = transport_.allocate(110, control_t);
    command.setBuffer(type_info.data_ptr);
    command.size = static_cast<uint8_t>(motor_ids_.size());
    for (i=0; i<joint_jog_->joint_names.size(); i++)
    {
      joint_name = joint_jog_->joint_names[i];
      if (!joint_names_2_index_.find(joint_name))
      {
        ROS_ERROR("Joint Name %s did not match any joint in the configuration", joint_name);
      }
      index = joint_names_2_index_[joint_name];
      x = joint_jog_->displacements

    }
    for (uint8_t i=0; i<motor_ids_.size(); i++)
    {
      x = [joint_jog_->joint_names]
      command.ids[i] = ;
    }
    if (send_position && has_control_data_)
    {
      
    }
    else
    {
      
    }
    

  }
  
  return has_feedback_data_;
}

void PfBoardControl::processDataFromBoard()
{
  if(!isInitialized())
  {
    ROS_ERROR("processDataFromBoard called when not initialized!");
  }

}

void PfBoardControl::transferDataToRos()
{
  if(!isInitialized())
  {
    ROS_ERROR("transferDataToRos called when not initialized!");
  }

}
