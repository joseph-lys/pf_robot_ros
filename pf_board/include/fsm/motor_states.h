/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_MOTOR_STATES_H
#define FSM_MOTOR_STATES_H

#include "fsm/base_classes.h"

namespace fsm
{


/// Transmit board configuration
class BootupState
{
}


/// Data is read from board but not sent to board
class PreloadState : public BaseState
{
  void sendToBoard(Data&); override
  void receiveFromBoard(Data&); override
  void sendToRos(Data&); override
  void receiveFromRos(Data&); override

  bool motorEnable();
  bool motorDisable();

  StatePtr nextState() override;
}

/// 
class StopState
{
};

/// 
class ErrorState 
{
};

}
#endif  // FSM_MOTOR_STATES_H