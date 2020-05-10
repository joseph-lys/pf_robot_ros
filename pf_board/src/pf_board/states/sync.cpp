#include "pf_board/states/wait.h"
#include "pf_board/states/sync.h"

using pf_board::states::Sync;
using pf_board::states::Stopped;
using pf_board::states::Error;


explicit Sync::Sync(IControl* p_control)
: BaseState<Sync>(p_control)
{
}

void Syc::stateEntry()
{
  delayRosError.setMillis(100);
  delayRosError.reset();
  delayBoardError.setMillis(100);
  delayBoardError.reset();
  delaySyncDone.setMillis(1000);
  delaySyncDone.reset();
  control_.setSrvIosWrite(false);
  control_.setSrvMotorCommand(false);
  control_.setSrvResetCommand(false);
  control_.setTorqueEnabled(false);
}

AbstractState* Sync::fromBoard()
{
  if (control_.transferFromBoard())
  {
    delayBoardError.reset();
  }
  return basePtr();
}

AbstractState* Sync::fromRos()
{
  if (control_.fromRos())
  {
    delayRosError.reset();
  }
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
    next = static_cast<AbstractState*>(new NonRecoverable{control_});
  }
  else if (delayRosError.isTimeout() || delayRosError.isTimeout())
  {
    /// try to sync all over again
    next = static_cast<AbstractState*>(new Sync{control_});
  }
  else if (control_.hasNoErrors() && delaySyncDone.isTimeout())
  {
    control_.setTorqueEnabled(true);  /// enable torque
    next = static_cast<AbstractState*>(new Run{control_});
  }
  return next;
}