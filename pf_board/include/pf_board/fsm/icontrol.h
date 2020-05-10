/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_FSM_ICONTROL_H
#define PF_BOARD_FSM_ICONTROL_H

namespace pf_board
{
namespace fsm
{

/// Interface class for providing controls to the state machine
class IControl
{
 public:
  ///////////////////////////////////////////////////////////////
  /// Communications control Functions
  ///////////////////////////////////////////////////////////////

  /// trigger reading data from subscribed ROS topics
  /// @return true if succeed, false otherwise
  virtual bool transferFromRos() = 0;

  /// trigger sending data to board
  /// @return true if succeed, false otherwise
  virtual bool transferToBoard() = 0;

  /// trigger reading data from board
  /// @return true if succeed, false otherwise
  virtual bool transferFromBoard() = 0;

  /// trigger publishing of ros data
  /// @return true if succeed, false otherwise
  virtual bool transferToRos() = 0;

  ///////////////////////////////////////////////////////////////
  /// Configuration Functions
  ///////////////////////////////////////////////////////////////  

  /// set the torque enabled
  /// @param enabled true will enable the torque on the motors
  /// false will disable the torque on the motors
  virtual void setTorqueEnabled(bool enabled) = 0;

  /// set the torque enabled
  /// @param value percentage * 100 or the max torque
  /// 10000 will be 100% max torque
  virtual void setTorqueMax(uint32_t value) = 0;

  ///////////////////////////////////////////////////////////////
  /// Service Configuration Functions
  ///////////////////////////////////////////////////////////////  

  /// set IO Write service availability
  /// @param availiable true to allow this service, false otherwise.
  virtual void setSrvIosWrite(bool availiable) = 0;

  /// set Motor Command service availability
  /// @param availiable true to allow this service, false otherwise.
  virtual void setSrvMotorCommand(bool availiable) = 0;

  /// set Reset Command service availability
  /// @param availiable true to allow this service, false otherwise.
  virtual void setSrvResetCommand(bool availiable) = 0;

  ///////////////////////////////////////////////////////////////
  /// Status Checking Functions
  ///////////////////////////////////////////////////////////////

  /// check if motor has errors, excluding communication errors
  /// @return true if there is errors, false otherwise
  virtual bool hasMotorErrors() = 0;

  /// check if there are non-recovarable errors
  /// @return true if there is any non-recovarable, false otherwise
  virtual bool hasNonRecoverableErrors() = 0;

  /// check if there are are not errors
  /// @return true if there is no active error, false otherwise
  virtual bool hasNoErrors() = 0;
};


}  // fsm
}  // pf_board
#endif  // PF_BOARD_FSM_ICONTROL_H
