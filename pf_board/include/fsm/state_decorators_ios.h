/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_STATE_DECORATORS_IOS_H
#define FSM_STATE_DECORATORS_IOS_H

#include "fsm/base_classes.h"
namespace pf_board
{
namespace fsm
{
namespace state_decorators_ios
{



class EnableBoardReadSensors : public BaseStateDecorator
{
 public:
  AbstractState* serviceCheckIORead(bool& service_available) override;
  AbstractState* receiveFromBoard(StateData& internal_state, DataFromBoard& data_from_broad) override;
  EnableBoardReadSensors(uint64_t start_delay_ns);
};

class EnableBoardWriteOutputs : public BaseStateDecorator
{
 public:
  AbstractState* serviceCheckIOWrite(bool& service_available) override;
  AbstractState* receiveFromBoard(StateData& internal_state, DataFromBoard& data_from_broad) override;
  EnableBoardWriteOutputs(uint64_t start_delay_ns);
};



}  // namespace state_decorators_ios
}  // namespace fsm
}  // namespace pf_board

#endif  // FSM_STATE_DECORATORS_MOTOR_H