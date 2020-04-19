/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_STATE_DECORATORS_EVENTS_H
#define FSM_STATE_DECORATORS_EVENTS_H

#include "fsm/base_classes.h"
namespace pf_board
{
namespace fsm
{
namespace state_decorators_events
{



class BoardTimeoutEvent : public pf_board::fsm::BaseStateDecorator
{
 public:
  AbstractState* receiveFromBoard(StateData& internal_state, DataFromBoard& data_from_board) override; 
  AbstractState* noReceiveFromBoard(StateData& internal_state) override; 
  BoardTimeoutEvent(uint64_t delay_ns=0);
};

class RosTimeoutEvent : public pf_board::fsm::BaseStateDecorator
{
 public:
  AbstractState* receiveFromRos(StateData& internal_state, DataFromRos& data_from_ros) override; 
  AbstractState* noReceiveFromRos(StateData& internal_state) override; 
  RosTimeoutEvent(uint64_t delay_ns=0);
};

template <typename T>
class ChangeAfterTime : public pf_board::fsm::BaseStateDecorator
{
 public:
  AbstractState* on(StateData& internal_state) override; 
  ChangeAfterTime(uint64_t delay_ns=0);
};



}  // namespace state_decorators_events
}  // namespace fsm
}  // namespace pf_board

#endif  // FSM_STATE_DECORATORS_EVENTS_H
