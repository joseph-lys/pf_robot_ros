/// Copyright 2020 Joseph Lee Yuan Sheng
///

#include "pf_board/states/base_state.h"
#include "pf_board/states/sync_state.h"
#include "pf_board/states/stop_state.h"


using pf_board::states::BaseState;
using pf_board::states::SyncState;
using pf_board::states::StopState;
using pf_board::states::IControl;

SyncState::SyncState()
{
  delay_ros_error_.setMillis(200);
  delay_board_error_.setMillis(100);
  delay_sync_done_.setMillis(5000);
}

std::string SyncState::getName()
{
  return std::string{"SyncState"};
}

BaseState* SyncState::executeLoop(IControl* p_control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  const bool send_position = true;
  const bool torque_enable = false;
  if (p_control->processDataFromRos())
  {
    delay_ros_error_.reset();
  }
  if (p_control->transferBoard(send_position, torque_enable))
  {
    delay_board_error_.reset();
    p_control->processDataFromBoard();
  }
  p_control->transferDataToRos();

  if (delay_ros_error_.isTimeout() || delay_board_error_.isTimeout())
  {
    delay_sync_done_.reset(); 
  }
  if (delay_sync_done_.isTimeout())
  {
    next_state = new StopState{};
  }
  return next_state;
}

void SyncState::enterState(IControl* p_control)
{
  delay_ros_error_.reset();
  delay_board_error_.reset();
  delay_sync_done_.reset();
}
