#include "pf_board/states/stop_state.h"
#include "pf_board/states/run_state.h"
#include "pf_board/states/error_state.h"


using pf_board::states::BaseState;
using pf_board::states::RunState;
using pf_board::states::StopState;
using pf_board::states::ErrorState;
using pf_board::states::IControl;

RunState::RunState(uint32_t auto_stop_ms)
{
  if (auto_stop_ms)
  {
    delay_auto_transition_.setMillis(auto_stop_ms);
    has_auto_transition_ = true;
  }
  else
  {
    has_auto_transition_ = false;
  }
  delay_ros_error_.setMillis(200);
  delay_board_error_.setMillis(100);
}

BaseState* RunState::executeLoop(IControl* control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  bool has_board_data;
  if (!control->processDataFromRos())
  {
    delay_ros_error_.reset();
  }
  has_board_data = control->transferBoard();
  if (has_board_data)
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
    next_state = new StopState{};
  }
  return next_state;
}

void RunState::enterState(IControl* control)
{
  delay_board_error_.reset();
  delay_ros_error_.reset();
  delay_auto_transition_.reset();

  control->setSrvIosWrite(true);
  control->setSrvMotorCommand(true);
  control->setSrvTorqueControlCommand(true);
  control->setSrvResetCommand(false);
  control->setPeriodicControl(true);

  control->setTorqueEnabled(true);
}

void RunState::exitState(IControl* control)
{
}
