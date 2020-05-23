/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_BASE_STATE_H
#define PF_BOARD_STATES_BASE_STATE_H

#include <string>
#include "pf_board/states/icontrol.h"

namespace pf_board
{
namespace states
{


class BaseState
{
 public:
  virtual ~BaseState() = default;

  virtual std::string getName() = 0;

  virtual BaseState* executeLoop(IControl*)
  {
    return this;
  }

  virtual BaseState* executeTorqueControl(IControl* p_control, bool torque_enable, uint32_t duration)
  { 
    return this;
  }

  virtual BaseState* executeResetCommand(IControl* p_control)
  {
    return this;
  }

  virtual void enterState(IControl*)
  {
    return;
  }

  virtual void exitState(IControl*)
  {
    return;
  }

};


}  // namespace states
}  // namespace pf_board

#endif  // PF_BOARD_STATES_BASE_STATE_H