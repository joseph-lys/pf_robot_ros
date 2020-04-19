/// Copyright 2020 Joseph Lee Yuan Sheng
///

#include "ros/ros.h"
#include "fsm/base_classes.h"

using pf_board::fsm::AbstractState;

using pf_board::fsm::BaseCompositeState;

AbstractState* BaseCompositeState::serviceCheckIOWrite(bool& service_available)
{
  return nullptr;
}
AbstractState* BaseCompositeState::serviceCheckIORead(bool& service_available)
{
  return nullptr;
}
AbstractState* BaseCompositeState::serviceCheckMotor(bool& service_available)
{
  return nullptr;
}
AbstractState* BaseCompositeState::receiveFromRos(BaseContext& internal_state, BaseContext& data_from_ros)
{
  return nullptr;
}
AbstractState* BaseCompositeState::noReceiveFromRos(BaseContext& internal_state)
{
  return nullptr;
}
AbstractState* BaseCompositeState::sendToBoard(BaseContext& internal_state, BaseContext& data_to_board)
{
  return nullptr;
}
AbstractState* BaseCompositeState::receiveFromBoard(BaseContext& internal_state, BaseContext& data_to_board)
{
  return nullptr;
}
AbstractState* BaseCompositeState::noReceiveFromBoard(BaseContext& internal_state)
{
  return nullptr;
}
AbstractState* BaseCompositeState::sendToRos(BaseContext& internal_state, BaseContext& data_to_board)
{
  return nullptr;
}
AbstractState* BaseCompositeState::nextState(BaseContext& internal_state)
{
  return nullptr;
}
void BaseCompositeState::onEntry()
{
}

using pf_board::fsm::BaseComponentState;
AbstractState* BaseComponentState::serviceCheckIOWrite(bool& service_available)
{
  AbstractState* next_state = nullptr;
  for(auto composite : composites_)
  {
    if (next_state == nullptr)
    {
      composite->serviceCheckIOWrite(service_available);
    }
    else
    {
      break;
    }
  }
  if (next_state == nullptr)
  {
    next_state = basePtr();
  }
  return next_state;
}
AbstractState* BaseComponentState::serviceCheckIORead(bool& service_available)
{
  AbstractState* next_state = nullptr;
  for(auto composite : composites_)
  {
    if (next_state == nullptr)
    {
      composite->serviceCheckIORead(service_available);
    }
    else
    {
      break;
    }
  }
  if (next_state == nullptr)
  {
    next_state = basePtr();
  }
  return next_state;
}
AbstractState* BaseComponentState::serviceCheckMotor(bool& service_available)
{
  AbstractState* next_state = nullptr;
  for(auto composite : composites_)
  {
    if (next_state == nullptr)
    {
      composite->serviceCheckMotor(service_available);
    }
    else
    {
      break;
    }
  }
  if (next_state == nullptr)
  {
    next_state = basePtr();
  }
  return next_state;
}
AbstractState* BaseComponentState::receiveFromRos(BaseContext& internal_state, BaseContext& data_from_ros)
{
  AbstractState* next_state = nullptr;
  for(auto composite : composites_)
  {
    if (next_state == nullptr)
    {
      composite->receiveFromRos(internal_state, data_from_ros);
    }
    else
    {
      break;
    }
  }
  if (next_state == nullptr)
  {
    next_state = basePtr();
  }
  return next_state;
}
AbstractState* BaseComponentState::noReceiveFromRos(BaseContext& internal_state)
{
  AbstractState* next_state = nullptr;
  for(auto composite : composites_)
  {
    if (next_state == nullptr)
    {
      composite->noReceiveFromRos(internal_state);
    }
    else
    {
      break;
    }
  }
  if (next_state == nullptr)
  {
    next_state = basePtr();
  }
  return next_state;
}
AbstractState* BaseComponentState::sendToBoard(BaseContext& internal_state, BaseContext& data_to_board)
{
  AbstractState* next_state = nullptr;
  for(auto composite : composites_)
  {
    if (next_state == nullptr)
    {
      composite->sendToBoard(internal_state, data_to_board);
    }
    else
    {
      break;
    }
  }
  if (next_state == nullptr)
  {
    next_state = basePtr();
  }
  return next_state;
}
AbstractState* BaseComponentState::receiveFromBoard(BaseContext& internal_state, BaseContext& data_from_board)
{
  AbstractState* next_state = nullptr;
  for(auto composite : composites_)
  {
    if (next_state == nullptr)
    {
      composite->receiveFromBoard(internal_state, data_from_board);
    }
    else
    {
      break;
    }
  }
  if (next_state == nullptr)
  {
    next_state = basePtr();
  }
}
AbstractState* BaseComponentState::noReceiveFromBoard(BaseContext& internal_state)
{
  AbstractState* next_state = nullptr;
  for(auto composite : composites_)
  {
    if (next_state == nullptr)
    {
      composite->noReceiveFromBoard(internal_state);
    }
    else
    {
      break;
    }
  }
  if (next_state == nullptr)
  {
    next_state = basePtr();
  }
}
AbstractState* BaseComponentState::sendToRos(BaseContext& internal_state, BaseContext& data_to_ros)
{
  AbstractState* next_state = nullptr;
  for(auto composite : composites_)
  {
    if (next_state == nullptr)
    {
      composite->sendToRos(internal_state, data_to_ros);
    }
    else
    {
      break;
    }
  }
  if (next_state == nullptr)
  {
    next_state = basePtr();
  }
}
AbstractState* BaseComponentState::nextState(BaseContext& internal_state)
{
  AbstractState* next_state = nullptr;
  for(auto composite : composites_)
  {
    if (next_state == nullptr)
    {
      composite->nextState(internal_state);
    }
    else
    {
      break;
    }
  }
  if (next_state == nullptr)
  {
    next_state = basePtr();
  }
}

BaseComponentState::BaseComponentState()
{

}

void BaseComponentState::onEntry()
{
  for(auto composite : composites_)
  {
    composite->onEntry();
  }
}
