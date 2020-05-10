/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_RUN_H
#define PF_BOARD_STATES_RUN_H

#include "pf_board/fsm/base_state.h"
#include "pf_board/simple_timer.h"

namespace pf_board
{
namespace states
{


/// Start producing recieved data, wait for some time so that data can propagate
class Run : public BaseState<Run>
{
 public:
  explicit Run();
  void configure() override;
  AbstractState* transitionFromRos(bool) override;
  AbstractState* transitionFromBoard(bool) override;
  AbstractState* transitionSequenceExit() override;
 private:
  SimpleTimer delayRosError;
  SimpleTimer delayBoardError;
  SimpleTimer delaySyncDone;
};



}  // states
}  // pf_board
#endif  // PF_BOARD_STATES_RUN_H