/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_STATES_ICONTROL_H
#define PF_BOARD_STATES_ICONTROL_H


#include <cstdint>

namespace pf_board
{
namespace states
{

/// Interface class for providing controls to the state machine
class IControl
{
 public:
  ///////////////////////////////////////////////////////////////
  /// Communications control Functions
  ///////////////////////////////////////////////////////////////

  /// trigger reading data from subscribed ROS topics, and process them
  /// @return true if data received and processed, false otherwise
  virtual bool processDataFromRos() = 0;

  /// trigger sending data exchange with board
  /// @return true if data was received from board, false otherwise
  virtual bool transferBoard() = 0;

  /// process board data
  virtual void processDataFromBoard() = 0;

  /// trigger publishing of ros data
  virtual void transferDataToRos() = 0;

  ///////////////////////////////////////////////////////////////
  /// State Manipulation Functions
  ///////////////////////////////////////////////////////////////  

  /// set the torque enabled
  /// @param enabled true will enable the torque on the motors
  /// false will disable the torque on the motors
  virtual void setTorqueEnabled(bool enabled) = 0;

  /// set all outputs to safestate
  virtual void setSafeStateIos() = 0;

  ///////////////////////////////////////////////////////////////
  /// Service Configuration Functions
  ///////////////////////////////////////////////////////////////  

  /// set IO Write service availability
  /// @param availiable true to allow this service, false otherwise.
  virtual void setSrvIosWrite(bool availiable) = 0;

  /// set Motor Command service availability
  /// @param availiable true to allow this service, false otherwise.
  virtual void setSrvMotorCommand(bool availiable) = 0;

  /// set Motor Torque Control Command service availability
  /// this service allows enabling/disable motor torque
  /// @param availiable true to allow this service, false otherwise.
  virtual void setSrvTorqueControlCommand(bool availiable) = 0;

  /// set Reset Command service availability
  /// @param availiable true to allow this service, false otherwise.
  virtual void setSrvResetCommand(bool availiable) = 0;

  /// set Period Control action
  /// @param enable true to enable periodic control, false otherwise.
  virtual void setPeriodicControl(bool enable) = 0;
};


}  // fsm
}  // pf_board
#endif  // PF_BOARD_STATES_ICONTROL_H
