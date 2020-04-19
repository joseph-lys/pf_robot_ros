/// Copyright 2020 Joseph Lee Yuan Sheng
///
#include "ros/ros.h"
#include "fsm/base_classes.h"

using pf_board::fsm::AbstractState;
using pf_board::fsm::BaseStateDecorator;


AbstractState* BaseStateDecorator::serviceCheckIOWrite(bool& service_available)
{
  return nullptr;
}
AbstractState* BaseStateDecorator::serviceCheckIORead(bool& service_available)
{
  return nullptr;
}
AbstractState* BaseStateDecorator::serviceCheckMotor(bool& service_available)
{
  return nullptr;
}
AbstractState* BaseStateDecorator::receiveFromRos(StateData& internal_state, DataFromRos& data_from_ros)
{
  return nullptr;
}
AbstractState* BaseStateDecorator::noReceiveFromRos(StateData& internal_state)
{
  return nullptr;
}
AbstractState* BaseStateDecorator::sendToBoard(StateData& internal_state, DataToBoard& data_to_board)
{
  return nullptr;
}
AbstractState* BaseStateDecorator::receiveFromBoard(StateData& internal_state, DataFromBoard& data_to_board)
{
  return nullptr;
}
AbstractState* BaseStateDecorator::noReceiveFromBoard(StateData& internal_state)
{
  return nullptr;
}
AbstractState* BaseStateDecorator::sendToRos(StateData& internal_state, DataToRos& data_to_board)
{
  return nullptr;
}
AbstractState* BaseStateDecorator::nextState(StateData& internal_state)
{
  return nullptr;
}
void BaseStateDecorator::onEntry()
{
}

using pf_board::fsm::BaseStatePrototype;
AbstractState* BaseStatePrototype::serviceCheckIOWrite(bool& service_available)
{
  service_available = false;
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
AbstractState* BaseStatePrototype::serviceCheckIORead(bool& service_available)
{
  service_available = false;
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
AbstractState* BaseStatePrototype::serviceCheckMotor(bool& service_available)
{
  service_available = false;
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
AbstractState* BaseStatePrototype::receiveFromRos(StateData& internal_state, DataFromRos& data_from_ros)
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
AbstractState* BaseStatePrototype::noReceiveFromRos(StateData& internal_state)
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
AbstractState* BaseStatePrototype::sendToBoard(StateData& internal_state, DataToBoard& data_to_board)
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
AbstractState* BaseStatePrototype::receiveFromBoard(StateData& internal_state, DataFromBoard& data_from_board)
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
AbstractState* BaseStatePrototype::noReceiveFromBoard(StateData& internal_state)
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
AbstractState* BaseStatePrototype::sendToRos(StateData& internal_state, DataToRos& data_to_ros)
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
AbstractState* BaseStatePrototype::nextState(StateData& internal_state)
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

BaseStatePrototype::BaseStatePrototype()
{

}

void BaseStatePrototype::onEntry()
{
  for(auto composite : composites_)
  {
    composite->onEntry();
  }
}
