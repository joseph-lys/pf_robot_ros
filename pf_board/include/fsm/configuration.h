/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef FSM_CONFIGURATION_H
#define FSM_CONFIGURATION_H

namespace pf_board
{
namespace fsm
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


}  // namespace fsm 
}  // namespace pf_board

#endif  // FSM_CONFIGURATION_H