#include "pf_board/states/wait.h"
#include "pf_board/states/sync.h"

using pf_board::states::Wait;
using pf_board::states::Sync;


explicit Wait::Wait(IControl& control)
: BaseState(control)
{
}

void Wait::stateEntry()
{
  delayRosOkay.setMillis(200);
  delayRosOkay.reset();
  delayBoardOkay.setMillis(200);
  delayBoardOkay.reset();
  control_.setSrvIosWrite(false);
  control_.setSrvMotorCommand(false);
  control_.setMaxTorque(0);
}

AbstractState* Wait::fromRos()
{
  if(!control_.transferFromRos())
  {
    delayRosOkay.reset();
  }
  return basePtr();
}

AbstractState* Wait::fromBoard()
{
  if(!control_.transferFromBoard())
  {
    delayBoardOkay.reset();
  }
  return basePtr();
}

AbstractState* Wait::sequenceExit()
{
  AbstractState* next = basePtr();
  if (delayRosOkay.isTimeout() && delayBoardOkay.isTimeout())
  {
    next = new Wait(control_);
  }
  return next;
}