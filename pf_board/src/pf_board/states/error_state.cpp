#include "pf_board/states/error_state.h"


using pf_board::states::BaseState;
using pf_board::states::ErrorState;
using pf_board::states::IControl;

ErrorState::ErrorState()
{
}

BaseState* ErrorState::executeLoop(IControl* control)
{
  BaseState* next_state = static_cast<BaseState*>(this);
  control->processDataFromRos();
  if (control->transferBoard())
  {
    control->processDataFromBoard();
    control->transferDataToRos();
  }

}

void ErrorState::enterState(IControl* control)
{
  control->setSrvIosWrite(false);
  control->setSrvMotorCommand(true);
  control->setSrvTorqueControlCommand(false);
  control->setPeriodicControl(false);
  control->setSrvResetCommand(true);

  control->setSafeStateIos();
  control->setTorqueEnabled(false);
}

void ErrorState::exitState(IControl* control)
{
}
