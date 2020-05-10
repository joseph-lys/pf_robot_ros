//////////////////////////////////////////////////////////////////////
/// Template member function definitions
//////////////////////////////////////////////////////////////////////
#include "pf_board/fsm/base_state.h"

using pf_board::fms::BaseState


void BaseState::configure()
{
  control_.mServiceCfgIosRead = false;
  control_.mServiceCfgIosWrite = false;
  control_.mServiceCfgMotorRead = false;
  control_.mServiceCfgMotorWrite = false;
}



AbstractState* BaseState::fromRos()
{
  return basePtr();
}

AbstractState* BaseState::toBoard()
{
  return basePtr();
}

AbstractState* BaseState::fromBoard()
{
  return basePtr();
}

AbstractState* BaseState::toRos()
{
  return basePtr();
}

AbstractState* BaseState::sequenceEntry()
{
  return basePtr();
}

AbstractState* BaseState::seqeunceExit()
{
  return basePtr();
}

void BaseState::stateEntry()
{
  return basePtr();
}

void BaseState::stateExit()
{
  return basePtr();
}


