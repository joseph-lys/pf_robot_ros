/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_ERROR_STATE_H
#define PF_BOARD_STATES_ERROR_STATE_H

#include "pf_board/states/base_state.h"
#include "pf_board/simple_timer.h"

namespace pf_board
{
namespace states
{


class ErrorState : public BaseState
{
 public:
  ErrorState();
  BaseState* executeLoop(IControl*) override;
  void enterState(IControl*) override;
  void exitState(IControl*) override;
};



}  // states
}  // pf_board
#endif  // PF_BOARD_STATES_ERROR_STATE_H