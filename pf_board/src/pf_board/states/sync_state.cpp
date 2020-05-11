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
  delay_ros_error.setMillis(200);
  delay_board_error.setMillis(100);
  delay_sync_done.setMillis(5000);
}

BaseState* SyncState::executeLoop(IControl* control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  if (control->processDataFromRos())
  {
    delay_ros_error.reset();
  }
  
  if (control->transferBoard())
  {
    delay_board_error.reset();
  }
  control->processDataFromBoard();
  control->transferDataToRos();

  if (delay_ros_error.isTimeout() || delay_board_error.isTimeout())
  {
    delay_sync_done.reset(); 
  }
  if (delay_sync_done.isTimeout())
  {
    next_state = new StopState;
  }
  return next_state;
}

void SyncState::enterState(IControl* control)
{
  delay_ros_error.reset();
  delay_board_error.reset();
  delay_sync_done.reset();

  control->setSrvIosWrite(false);
  control->setSrvMotorCommand(false);
  control->setSrvTorqueControlCommand(false);
  control->setSrvResetCommand(false);
  control->setPeriodicControl(false);

  control->setSafeStateIos();
}

void SyncState::exitState(IControl* control)
{
}