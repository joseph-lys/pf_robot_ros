/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_STOP_STATE_H
#define PF_BOARD_STATES_STOP_STATE_H

#include "pf_board/states/base_state.h"
#include "pf_board/simple_timer.h"

namespace pf_board
{
namespace states
{

/// Outputs and Motors are disabled until start signal is received
class StopState : public BaseState
{
 public:
  explicit StopState(uint32_t auto_start_ms=0);
 private:
  BaseState* executeLoop(IControl*) override;
  void enterState(IControl*) override;
  void exitState(IControl*) override;

  bool has_auto_transition_;
  pf_board::SimpleTimer delay_auto_transition_;
  pf_board::SimpleTimer delay_board_error_;
  pf_board::SimpleTimer delay_ros_error_;
};



}  // states
}  // pf_board
#endif  // PF_BOARD_STATES_STOP_STATE_H