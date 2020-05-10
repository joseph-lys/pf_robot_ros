/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_ABSTRACT_STATE_H
#define FSM_ABSTRACT_STATE_H

#include "pf_board/fsm/icontrol.h"

namespace pf_board
{
namespace fsm
{

/// State base class
/// The functions will be called in this sequence
/// scanEntry() -> [OP] -> fromRos() -> [OP] -> toBoard() -> [OP] -> fromBoard() -> [OP] -> toRos() -> [OP] -> scanExit()
///
class AbstractState
{
 public:
  /// This function is called at the start of each loop
  virtual AbstractState* sequenceEntry() = 0;

  /// This function is called to get data from ROS
  virtual AbstractState* fromRos() = 0;

  /// This function is called to send data to Board
  virtual AbstractState* toBoard() = 0;

  /// This function is called to get data from Board
  virtual AbstractState* fromBoard() = 0;

  /// This function is called to send data to ROS
  virtual AbstractState* toRos() = 0;

  /// This function is called at the end of each loop
  virtual AbstractState* sequenceExit() = 0;

  /// This function is called before entering a state
  virtual AbstractState* stateEntry() = 0;

  /// This function is called before leaving a state
  virtual AbstractState* stateExit() = 0;

  virtual ~AbstractState() = default;
 protected:
  AbstractState() = default;
};



} // namespace fsm
} // namespace pf_board

#endif  // FSM_ABSTRACT_STATE_H
