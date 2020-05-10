/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_WAIT_H
#define PF_BOARD_STATES_WAIT_H

#include "pf_board/fsm/base_state.h"
#include "pf_board/simple_timer.h"

namespace pf_board
{
namespace states
{


/// Wait for board and ros to both produce data
class Wait : private BaseState
{
 public:
  explicit Wait(IControl&);
 private:
  AbstractState* fromRos() override;
  AbstractState* fromBoard() override;
  AbstractState* sequenceExit() override;
  void stateEntry() override;
  SimpleTimer delayRosOkay;
  SimpleTimer delayBoardOkay;
};



}  // states
}  // pf_board
#endif  // PF_BOARD_STATES_WAIT_H