#include "pf_board/states/run_state.h"
#include "pf_board/states/stop_state.h"
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

std::string RunState::getName()
{
  return std::string{"RunState"};
}

BaseState* RunState::executeLoop(IControl* p_control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  const bool send_position = true;
  const bool torque_enable = true;
  if (p_control->processDataFromRos())
  {
    delay_ros_error_.reset();
  }
  if (p_control->transferBoard(send_position, torque_enable))
  {
    delay_board_error_.reset();
    p_control->processDataFromBoard();
    p_control->transferDataToRos();
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

BaseState* RunState::executeTorqueControl(IControl* p_control, bool torque_enable, uint32_t duration)
{
  BaseState* next = static_cast<BaseState*>(this);
  if (!torque_enable)
  {
    next = new StopState{duration};
  }
  return next;
}

void RunState::enterState(IControl* p_control)
{
  delay_board_error_.reset();
  delay_ros_error_.reset();
  delay_auto_transition_.reset();
}
