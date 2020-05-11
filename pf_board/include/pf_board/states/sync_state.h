/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_SYNC_STATE_H
#define PF_BOARD_STATES_SYNC_STATE_H

#include "pf_board/states/base_state.h"
#include "pf_board/simple_timer.h"

namespace pf_board
{
namespace states
{


/// Start producing recieved data, wait for some time so that data can propagate
class SyncState : public BaseState
{
 public:
  SyncState();
  BaseState* executeLoop(IControl*) override;
  void enterState(IControl*) override;
  void exitState(IControl*) override;
 private:
  pf_board::SimpleTimer delay_ros_error;
  pf_board::SimpleTimer delay_board_error;
  pf_board::SimpleTimer delay_sync_done;
};



}  // states
}  // pf_board
#endif  // PF_BOARD_STATES_SYNC_STATE_H