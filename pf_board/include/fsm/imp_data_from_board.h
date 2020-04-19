/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_IMP_DATA_FROM_BOARD_H
#define FSM_IMP_DATA_FROM_BOARD_H

#include "fsm/base_classes.h"

namespace pf_board
{
namespace fsm
{

/// internal struct, will be replaced in future
struct InternalDataFromBoard_
{ 
  uint32_t system_time_us;

  // Motor
  uint16_t position[kMaxMotors];
  uint16_t speed[kMaxMotors];
  uint16_t torque[kMaxMotors];
  uint8_t status[kMaxMotors];
  bool enabled[kMaxMotors];
  
  // I/O
  uint16_t analog_in[kMaxAI];
  bool digital_in[kMaxDI];

  // IMU
  bool has_mag[kMaxIMUs];
  uint16_t accel_x[kMaxIMUs]; 
  uint16_t accel_y[kMaxIMUs]; 
  uint16_t accel_z[kMaxIMUs]; 
  uint16_t gyro_x[kMaxIMUs]; 
  uint16_t gyro_y[kMaxIMUs]; 
  uint16_t gryo_z[kMaxIMUs]; 
  uint16_t mag_x[kMaxIMUs]; 
  uint16_t mag_y[kMaxIMUs]; 
  uint16_t mag_z[kMaxIMUs];
};

struct ImpDataFromBoard : public DataFromBoard
{
  InternalDataFromBoard_ data;
};

}  // namespace fsm 
}  // namespace pf_board

#endif  // FSM_IMP_DATA_FROM_BOARD_H