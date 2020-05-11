/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_BASE_STATE_H
#define PF_BOARD_STATES_BASE_STATE_H

#include "pf_board/states/icontrol.h"

namespace pf_board
{
namespace states
{


class BaseState
{
 public:
  virtual ~BaseState() = default;

  virtual BaseState* executeLoop(IControl*) = 0;

  virtual void enterState(IControl*) = 0;

  virtual void exitState(IControl*) = 0;
};


}  // namespace states
}  // namespace pf_board

#endif  // PF_BOARD_STATES_BASE_STATE_H