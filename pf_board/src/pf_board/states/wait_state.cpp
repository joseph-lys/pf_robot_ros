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
  delay_ros_error.setMillis(200);  // must receive at least 5 Hz 
  delay_board_okay.setMillis(100);  // must receive at least 10 Hz
  delay_wait_done.setMillis(1000);
}

BaseState* WaitState::executeLoop(IControl* control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  if (!control->processDataFromRos())
  {
    delay_ros_error.reset();
  }
  if (control->transferBoard())
  {
    delay_board_okay.reset();
  }
  if (!delay_ros_error.isTimeout() && !delay_board_okay.isTimeout() && delay_wait_done.isTimeout())
  {
    next_state = new SyncState;
  }
  return next_state;
  
}

void WaitState::enterState(IControl* control)
{
  delay_ros_error.reset();
  delay_board_okay.reset();
  delay_wait_done.reset();

  control->setSrvIosWrite(false);
  control->setSrvMotorCommand(false);
  control->setSrvTorqueControlCommand(false);
  control->setSrvResetCommand(false);
  control->setPeriodicControl(false);

  control->setSafeStateIos();
}

void WaitState::exitState(IControl* control)
{
}
