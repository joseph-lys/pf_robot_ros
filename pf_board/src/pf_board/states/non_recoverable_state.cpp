#include "pf_board/states/non_recoverable_state.h"


using pf_board::states::BaseState;
using pf_board::states::NonRecoverableState;
using pf_board::states::IControl;


std::string NonRecoverableState::getName()
{
  return std::string{"NonRecoverableState"};
}

BaseState* NonRecoverableState::executeLoop(IControl* p_control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  const bool send_position = false;
  const bool torque_enable = false;
  p_control->processDataFromRos();
  if (p_control->transferBoard(send_position, torque_enable))
  {
    p_control->processDataFromBoard();
  }
  p_control->transferDataToRos();
}
