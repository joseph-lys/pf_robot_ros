/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_IMP_DATA_TO_BOARD_H
#define FSM_IMP_DATA_TO_BOARD_H

#include "fsm/base_classes.h"

namespace pf_board
{
namespace fsm
{

/// internal struct, will be replaced in future
struct InternalDataToBoard_
{ 
  // Motor
  uint16_t position[kMaxMotors];
  uint16_t speed[kMaxMotors];
  bool enabled[kMaxMotors];

  // I/O
  bool digital_out[kMaxDO];
};

struct ImpDataToBoard : public DataToBoard
{
  InternalDataToBoard_ data;
};

}  // namespace fsm 
}  // namespace pf_board

#endif  // FSM_IMP_DATA_TO_BOARD_H