#include "pf_board/states/wait.h"
#include "pf_board/states/sync.h"

using pf_board::states::Sync;
using pf_board::states::Run;
using pf_board::states::Error;


explicit Sync::Sync(IControl* p_control)
: BaseState<Sync>(p_control)
{
}

void Syc::stateEntry()
{
  delayResetAllowed.setMillis(1000);
  delayResetAllowed.reset();
  control_.setSrvIosWrite(false);
  control_.setSrvMotorCommand(false);
  control_.setSrvResetCommand(false);
  control_.setTorqueEnabled(false);
}

AbstractState* Sync::fromRos()
{
  control_.transferFromRos();
  return basePtr();
}

AbstractState* Sync::fromBoard()
{
  control_.transferFromBoard();
  return basePtr();
}

AbstractState* Sync::toBoard()
{
  control_.transferToBoard();
  return basePtr();
}

AbstractState* Sync::toRos()
{
  control_.transferToRos();
  return basePtr();
}

AbstractState* Sync::sequenceExit()
{
  AbstractState* next = basePtr();
  if (control_.hasNonRecoverableErrors())
  {
    next = static_cast<AbstractState*>(new Stopped{control_});
  }
  else if (!control_.hasNoErrors())
  {
    delayResetAllowed.reset();
  }
  else if (delayResetAllowed.isTimeout() && control_.hasNoErrors())
  {
    next = static_cast<AbstractState*>(new Sync{control_});
  }
  return next;
}