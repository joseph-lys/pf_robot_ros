#include "pf_board/states/idle.h"
#include "pf_board/states/wait.h"

using pf_board::states::Idle;
using pf_board::states::Wait;

explicit Idle::Idle(IControl& control)
: BaseState(control)
{
}

void Idle::stateEntry()
{
  delayToConfigState.setMillis(3000);
  delayToConfigState.reset();
  control_.setSrvIosWrite(false);
  control_.setSrvMotorCommand(false);
  control_.setMaxTorque(0);
}

AbstractState* Idle::sequenceExit()
{
  AbstractState* next = basePtr();
  if (delayToConfigState.isTimeout())
  {
    next = new Wait(control_);
  }
  return next;
}