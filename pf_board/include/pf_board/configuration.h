/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_CONFIGURATION_H
#define PF_BOARD_CONFIGURATION_H

namespace pf_board
{
/// Configuration Constants
enum ConfigurationConstants: uint8_t 
{
  kMaxMotors = 32,
  kMaxAI = 8,
  kMaxDI = 2,
  kMaxDO = 1,
  kMaxIMUs = 2
};


}  // namespace pf_board

#endif  // PF_BOARD_CONFIGURATION_H