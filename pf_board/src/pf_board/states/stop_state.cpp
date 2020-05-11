/// Copyright 2020 Joseph Lee Yuan Sheng
///
#include "pf_board/states/base_state.h"
#include "pf_board/states/stop_state.h"
#include "pf_board/states/run_state.h"
#include "pf_board/states/error_state.h"

using pf_board::states::BaseState;
using pf_board::states::StopState;
using pf_board::states::RunState;
using pf_board::states::ErrorState;

StopState::StopState(uint32_t auto_start_ms)
{
  if (auto_start_ms)
  {
    delay_auto_transition_.setMillis(auto_start_ms);
    has_auto_transition_ = true;
  }
  else
  {
    has_auto_transition_ = false;
  }
}

BaseState* StopState::executeLoop(IControl* control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  if (!control->processDataFromRos())
  {
    delay_ros_error_.reset();
  }
  if (control->transferBoard())
  {
    delay_board_error_.reset();
    control->processDataFromBoard();
    control->transferDataToRos();
  }

  if (delay_ros_error_.isTimeout() || delay_board_error_.isTimeout())
  {
    next_state = new ErrorState{};
  }
  else if (has_auto_transition_ && delay_auto_transition_.isTimeout())
  {
    next_state = new RunState{};
  }
  return next_state;
}

void StopState::enterState(IControl* control)
{
  delay_auto_transition_.reset();
  delay_board_error_.reset();
  delay_ros_error_.reset();

  control->setSrvIosWrite(true);
  control->setSrvMotorCommand(true);
  control->setSrvTorqueControlCommand(true);
  control->setSrvResetCommand(false);
  control->setPeriodicControl(true);

  control->setSafeStateIos();
  control->setTorqueEnabled(false);
}

void StopState::exitState(IControl* control)
{
}
