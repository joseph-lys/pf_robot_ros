/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_DEFINED_STATES_H
#define FSM_DEFINED_STATES_H

#include "fsm/base_classes.h"
namespace pf_board
{
namespace fsm
{

/// Idle State for initialization
class IdleState : public BaseState <IdleState>
{
  friend BaseState;
 protected:
  IdleState();
};

/// Configuring the board
class ConfigState : public BaseState <ConfigState>
{
  friend BaseState;
 protected:
  ConfigState();
};

/// Preload State, allow some time for information to propagate properly in the system
class PreloadState : public BaseState <PreloadState>
{
  friend BaseState;
 protected:
  PreloadState();
};

/// Run State, all components are running normally
class RunState : public BaseState <RunState>
{
  friend BaseState;
 protected:
  RunState();
};

/// Stop State, all components are running normally, motors are stopped intentionally
class StopState : public BaseState <StopState>
{
  friend BaseState;
 protected:
  StopState();
};

/// ErrorState, some critical error has occured.
class ErrorState : public BaseState <ErrorState>
{
  friend BaseState;
 protected:
  ErrorState();
};



}  // namespace fsm
}  // namesapce pf_board
#endif  // FSM_DEFINED_STATES_H