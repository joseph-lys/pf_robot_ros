/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_STATE_DECORATORS_MOTORS_H
#define FSM_STATE_DECORATORS_MOTORS_H

#include "fsm/base_classes.h"
namespace pf_board
{
namespace fsm
{
namespace state_decorators_motors
{



class EnableBoardRead : public BaseStateDecorator
{
 public:
  AbstractState* receiveFromBoard(StateData& internal_state, DataFromBoard& data_from_board) override;
  EnableBoardRead(uint64_t start_delay_ns);
};

class EnableBoardWrite : public BaseStateDecorator
{
 public:
  AbstractState* serviceCheckMotor(bool& service_available);
  AbstractState* sendToBoard(StateData& internal_state, DataToBoard& data_to_board) override;
  EnableBoardWrite(uint64_t start_delay_ns);
};

/// Enable motor Torque on entry
class EnableMotorTorqueOnEntry : public BaseStateDecorator
{
 public:
  AbstractState* sendToBoard(StateData& internal_state, DataToBoard& data_to_board) override;
  EnableMotorTorqueOnEntry(uint64_t on_duration_ns);
};

class DisableMotorTorque : public BaseStateDecorator
{
 public:
  AbstractState* sendToBoard(StateData& internal_state, DataToBoard& data_to_board) override;
  DisableMotorTorque(uint64_t start_delay_ns);
};

class PublishMotorData : public BaseStateDecorator
{
 public:
  AbstractState* sendToRos(StateData& internal_state, DataToRos& data_to_ros) override;
  PublishMotorData(uint64_t start_delay_ns);
};

class SubcribeMotorData : public BaseStateDecorator
{
 public:
  AbstractState* receiveFromRos(StateData& internal_state, DataFromRos& data_from_ros) override; 
  SubcribeMotorData(uint64_t start_delay_ns);
};




}  // namespace state_decorators_motors
}  // namespace fsm
}  // namespace pf_board

#endif  // FSM_STATE_DECORATORS_MOTORS_H