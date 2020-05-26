/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_STATE_MACHINE_H
#define PF_BOARD_STATES_RUN_STATE_H

#include "pf_board/states/icontrol.h"
#include "pf_board/states/base_state.h"

namespace pf_board
{
namespace states
{



class StateMachine
{
 public:
  ~StateMachine();
  void setControl(pf_board::states::IControl* p_contol);
  bool srvTorqueControl(bool torque_enabled);
  bool srvResetCommand();
  void executeLoop();

 private:
  pf_board::states::IControl* p_control_ = nullptr;
  pf_board::states::BaseState* p_state_ = nullptr;
  bool updateState(pf_board::states::BaseState* p_next_state);
};




}  // states
}  // pf_board
#endif  // PF_BOARD_STATES_STATE_MACHINE_H
