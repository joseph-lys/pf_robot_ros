/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_IDLE_H
#define PF_BOARD_STATES_IDLE_H

#include "pf_board/fsm/base_state.h"
#include "pf_board/simple_timer.h"

namespace pf_board
{
namespace states
{


class Idle : public BaseState<Idle>
{
 public:
  explicit Idle(IControl&);
 private:
  AbstractState* sequenceEntry() override;
  AbstractState* stateEntry() override;
  SimpleTimer delayToConfigState;
};



}  // states
}  // pf_board
#endif  // PF_BOARD_FSM_ICONTROL_H