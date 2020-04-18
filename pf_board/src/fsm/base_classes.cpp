#include "fsm/base_classes.h"

using fsm::BaseState;

BaseState::BaseState()
{
}

BaseState::~BaseState()
{
}

void BaseState::receiveFromRos(ContextPtr)
{
}
  
void BaseState::sendToBoard(ContextPtr)
{
}

void BaseState::receiveFromBoard(ContextPtr)
{
}

void BaseState::sendToRos(ContextPtr)
{
}

std::shared_ptr<BaseState> BaseState::nextState()
{
  return std::shared_ptr<BaseState>{};
}

