/// Copyright 2020 Joseph Lee Yuan Sheng
///

#include "pf_board/states/base_state.h"
#include "pf_board/states/wait_state.h"
#include "pf_board/states/sync_state.h"

using pf_board::states::BaseState;
using pf_board::states::WaitState;
using pf_board::states::IControl;

WaitState::WaitState()
{
  delay_ros_error_.setMillis(200);  // must receive at least 5 Hz 
  delay_board_okay_.setMillis(100);  // must receive at least 10 Hz
  delay_wait_done_.setMillis(1000);
}

std::string WaitState::getName()
{
  return std::string{"WaitState"};
}

BaseState* WaitState::executeLoop(IControl* p_control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  const bool send_position = true;
  const bool torque_enable = false;
  if (!p_control->processDataFromRos())
  {
    delay_ros_error_.reset();
  }
  if(!p_control->transferBoard(send_position, torque_enable))
  {
    delay_board_okay_.reset();
  }
  if (!delay_ros_error_.isTimeout() && !delay_board_okay_.isTimeout() && delay_wait_done_.isTimeout())
  {
    next_state = static_cast<BaseState*>(new SyncState{});
  }
  return next_state;
  
}

void WaitState::enterState(IControl* control)
{
  delay_ros_error_.reset();
  delay_board_okay_.reset();
  delay_wait_done_.reset();
}
