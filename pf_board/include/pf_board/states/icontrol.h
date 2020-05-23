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
  /// @param send_position will send command to update motor position if true
  /// @param torque_enable enable torque on motor if true, otherwise disable torque
  /// @return true if data was received from board, false otherwise
  virtual bool transferBoard(bool send_position, bool torque_enable) = 0;

  /// process board data
  virtual void processDataFromBoard() = 0;

  /// trigger publishing of ros data
  virtual void transferDataToRos() = 0;
};


}  // fsm
}  // pf_board
#endif  // PF_BOARD_STATES_ICONTROL_H
