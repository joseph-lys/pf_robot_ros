/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_ABSTRACT_STATE_H
#define FSM_ABSTRACT_STATE_H

#include <vector>
#include <memory>

namespace pf_board
{
namespace fsm
{
/// State base class
class BaseState : private AbstractState
{
 protected:
  /// general configuration
  explicit configure();
  /// protected constructor, only subclasses can create instance
  explicit BaseState(IControl&);
  /// returns this pointer casted as AbstractState*
  AbstractState* basePtr();

  /// reference to IControl object
  IControl& control_;
 private:
  AbstractState* sequenceEntry() override;
  AbstractState* fromRos() override;
  AbstractState* toBoard() override;
  AbstractState* fromBoard() override;
  AbstractState* toRos() override;
  AbstractState* sequenceExit() override;
  void stateEntry() override;
  void stateExit() override;
};



} // namespace fsm
} // namespace pf_board



#include "pf_board/fsm/base_state.tcc"  // include the implementation

#endif  // FSM_ABSTRACT_STATE_H