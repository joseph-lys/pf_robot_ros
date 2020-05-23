/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_WAIT_STATE_H
#define PF_BOARD_STATES_WAIT_STATE_H

#include "pf_board/states/base_state.h"
#include "pf_board/simple_timer.h"

namespace pf_board
{
namespace states
{


/// Wait for board and ros to both produce data
class WaitState : private BaseState
{
 public:
  WaitState();
  std::string getName() override;
  BaseState* executeLoop(IControl*) override;
  void enterState(IControl*) override;
 private:
  pf_board::SimpleTimer delay_ros_error_;
  pf_board::SimpleTimer delay_board_okay_;
  pf_board::SimpleTimer delay_wait_done_;
};



}  // states
}  // pf_board
#endif  // PF_BOARD_STATES_WAIT_STATE_H