#include "pf_board/states/state_machine.h"
#include "pf_board/states/wait_state.h"

using pf_board::states::StateMachine;
using pf_board::states::BaseState;
using pf_board::states::IControl;
using pf_board::states::WaitState;

StateMachine::StateMachine()
{
  p_state_ = static_cast<BaseState*>(new WaitState{});
}

StateMachine::~StateMachine()
{
  if (p_state_ != nullptr)
  {
    p_state->exitState(p_control);
    delete p_state_;
  }
}

bool StateMachine::updateState(BaseState* p_next_state)
{
  std::string old_state = "NullState";
  std::string new_state = "NullState";

  bool updated = false;
  if (p_control_ == nullptr)
  {
    ROS_ASSERT("Tried to update state before control set");
  }
  else if (p_state_ != p_next_state)
  {
    updated = true;
    if (p_state_ != nullptr)
    {
      p_state->exitState(p_control);
      delete p_state_;
    }
    p_state_ = p_next_state;
    if (p_state_ != nullptr)
    {
      p_state_->enterState(p_control);
    }
    ROS_DEBUG("State transition: %s --> %s", old_state.c_str(), new_state.c_str());
  }
  return updated;
}

void StateMachine::setControl(IControl* p_contol)
{
  p_control_ = p_control;
}

bool StateMachine::srvTorqueControl(bool torque_enabled, uint32_t duration)
{
  bool success = false;
  if (p_control_ == nullptr)
  {
    ROS_DEBUG("Tried to call Torque Control service when control not set");
  }
  else
  {
    success = updateState(p_state_->executeTorqueControl(p_control_, torque_enable, duration));
    if (!success)
    {
      ROS_DEBUG("Torque Control Service request is not valid for current state");
    }
  }
  return success;
}


bool StateMachine::srvResetCommand()
{
  bool success = false;
  if (p_control_ == nullptr)
  {
    ROS_DEBUG("Tried to call Reset Command action when control not set");
  }
  else
  {
    success = updateState(p_state_->executeResetCommand());
    if (!success)
    {
      ROS_DEBUG("Reset Command Service request is not valid for current state");
    }
  }
  return success;
}

void StateMachine::executeLoop()
{
  if (p_state_ == nullptr)
  {
    ROS_ASSERT("Tried to run Loop when state not initialized");
  }
  else
  {
    updateState(->executeLoop());
  }
}
