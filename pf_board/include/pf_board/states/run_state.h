/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_RUN_STATE_H
#define PF_BOARD_STATES_RUN_STATE_H

#include "pf_board/states/base_state.h"
#include "pf_board/simple_timer.h"

namespace pf_board
{
namespace states
{

class RunState : public BaseState
{
 public:
  explicit RunState(uint32_t auto_stop_ms=0);
  std::string getName() override;
 private:
  BaseState* executeLoop(IControl*) override;
  BaseState* executeTorqueControl(IControl* p_control, bool torque_enable, uint32_t duration) override;
  void enterState(IControl*) override;

  bool has_auto_transition_;
  pf_board::SimpleTimer delay_auto_transition_;
  pf_board::SimpleTimer delay_board_error_;
  pf_board::SimpleTimer delay_ros_error_;
};



}  // states
}  // pf_board
#endif  // PF_BOARD_STATES_RUN_STATE_H
