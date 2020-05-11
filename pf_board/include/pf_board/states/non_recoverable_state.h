/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_NON_RECOVERABLE_STATE_H
#define PF_BOARD_STATES_NON_RECOVERABLE_STATE_H

#include "pf_board/states/base_state.h"
#include "pf_board/simple_timer.h"

namespace pf_board
{
namespace states
{


/// Wait for board and ros to both produce data
class NonRecoverableState : public BaseState
{
 public:
  NonRecoverableState();
  BaseState* executeLoop(IControl*) override;
  void enterState(IControl*) override;
  void exitState(IControl*) override;
};



}  // states
}  // pf_board
#endif  // PF_BOARD_STATES_NON_RECOVERABLE_STATE_H