#include "pf_board/states/run.h"
#include "pf_board/states/error.h"
#include "pf_board/states/stop.h"


using pf_board::states::Run;
using pf_board::states::Error;


explicit Run::Run(IControl* p_control)
: BaseState<Run>(p_control)
{
  delayRosError.setMillis(100);
  delayBoardError.setMillis(100);
}

void Sync::configure()
{
  p_control_->mServiceCfgIosRead = = delayIosService.isTimeout() ? true : false;
  p_control_->mServiceCfgIosWrite = delayIosService.isTimeout() ? true : false;
  p_control_->mServiceCfgMotorRead = delayMotorService.isTimeout() ? true : false;
  p_control_->mServiceCfgMotorWrite = delayMotorService.isTimeout() ? true : false;

  p_control_->mTransferCfgFromRos = true;
  p_control_->mTransferCfgToBoard = delaySendBoard.isTimeout() ? true : false;  
  p_control_->mTransferCfgFromBoard = true;
  p_control_->mTransferCfgToRos = true;
}

AbstractState* Sync::transitionFromBoard(bool received)
{
  if (received)
  {
    delayBoardError.reset();
  }
  return basePtr();
}

AbstractState* Sync::transitionFromRos(bool received)
{
  if (received)
  {
    delayRosError.reset();
  }
  return basePtr();
}

AbstractState* Sync::transitionSequenceExit()
{
  AbstractState* next = basePtr();
  if (delayRosError.isTimeout() || delayRosError.isTimeout())
  {
    next = Sync::nextState(basePtr());
  }
  else if (delaySyncDone.isTimeout())
  {
    next = Run::nextState(basPtr());
  }
  return next;
}