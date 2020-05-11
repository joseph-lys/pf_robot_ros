#include "pf_board/states/non_recoverable_state.h"


using pf_board::states::BaseState;
using pf_board::states::NonRecoverableState;
using pf_board::states::IControl;

NonRecoverableState::NonRecoverableState()
{
}

BaseState* NonRecoverableState::executeLoop(IControl* control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  control->processDataFromRos();
  if (control->transferBoard())
  {
    control->processDataFromBoard();
    control->transferDataToRos();
  }
}

void NonRecoverableState::enterState(IControl* control)
{
  control->setSrvIosWrite(false);
  control->setSrvMotorCommand(false);
  control->setSrvTorqueControlCommand(false);
  control->setPeriodicControl(false);
  control->setSrvResetCommand(false);

  control->setSafeStateIos();
  control->setTorqueEnabled(false);
}

void NonRecoverableState::exitState(IControl* control)
{
}
