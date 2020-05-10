/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_SYNC_H
#define PF_BOARD_STATES_SYNC_H

#include "pf_board/fsm/base_state.h"
#include "pf_board/simple_timer.h"

namespace pf_board
{
namespace states
{


/// Start producing recieved data, wait for some time so that data can propagate
class Sync : public BaseState
{
 public:
  explicit Sync(IControl&);
 private:
  AbstractState* fromRos() override;
  AbstractState* toBoard()) override;
  AbstractState* fromBoard() override;
  AbstractState* toRos() override;
  AbstractState* sequenceExit() override;
  void stateEntry() override;
  SimpleTimer delayRosError;
  SimpleTimer delayBoardError;
  SimpleTimer delaySyncDone;
};



}  // states
}  // pf_board
#endif  // PF_BOARD_STATES_SYNC_H