#include "pf_board/states/error_state.h"
#include "pf_board/states/stop_state.h"

using pf_board::states::BaseState;
using pf_board::states::ErrorState;
using pf_board::states::IControl;
using pf_board::states::StopState;


std::string ErrorState::getName()
{
  return std::string{"ErrorState"};
}

BaseState* ErrorState::executeLoop(IControl* p_control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  const bool send_position = true;
  const bool torque_enable = false;
  p_control->processDataFromRos();
  if (p_control->transferBoard(send_position, torque_enable))
  {
    p_control->processDataFromBoard();
  }
  p_control->transferDataToRos();
}

BaseState* ErrorState::executeResetCommand(IControl* p_control)
{
  return static_cast<BaseState*>(new StopState{});
}
