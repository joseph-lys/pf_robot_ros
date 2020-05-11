/// Copyright 2020 Joseph Lee Yuan Sheng
///
/// very simple transport layer
///
/// 
/// Header:
///  Byte 0         : len    # number of structs
/// Struct 1:
///  Byte X         : type_info
///  Byte X+1       : bytes  # higher
///  Byte X+4...    : data
/// Struct N :
///  Byte Y         : type_info
///  Byte Y+1       : bytes  # higher
///  Byte Y+4...    : data
/// CRC16
///  Byte Z         : chksum # lower
///  Byte Z+1       : chksum # higher

#ifndef PF_BOARD_COMMS_TRANSPORT_LAYER_H
#define PF_BOARD_COMMS_TRANSPORT_LAYER_H

#include <cstdint>

namespace pf_board
{
namespace transport_layer
{


struct TypeInfo
{
  uint8_t* data_ptr;
  uint16_t data_len;
  uint8_t type_enum;
};

class TransportLayer
{
 public:
  enum TypeEnum : uint8_t
  {
    control_t=1,
    feedback_t,
    motor_cmd_request_t,
    motor_cmd_response_t,
  };
  //// Functions for read operation

  /// Start Reading from target buffer
  /// @returns number of structs available 
  uint8_t readFrom(uint8_t*);

  /// Get typeinfo of next struct, does not move the read position
  TypeInfo peek();

  /// Get typeinfo of the next struct, move the read position
  TypeInfo next();


  //// Functions for write operation

  /// Start Writing to target buffer
  void writeTo(uint8_t*);

  /// Allocate N amount of data
  TypeInfo allocate(uint8_t len, uint8_t type_enum=0);

  /// Copy struct data from a buffer
  bool addCopy(uint8_t* source_data, uint8_t len, uint8_t type_enum=0);

  /// Finalize all data 
  bool finalizeWrite();

  /// Resets to the initial write state
  void clear();

 private:
  bool readable_ = false;
  bool writable_ = false;
  bool byte_index_ = 0;
  bool struct_index_ = 0;
  uint8_t* buf_ = nullptr;
};


}  // namespace transport_layer
}  // namespace pf_board

#endif  // PF_BOARD_COMMS_TRANSPORT_LAYER_H
