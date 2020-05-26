/// Copyright 2020 Joseph Lee Yuan Sheng
///
/// very simple transport layer
///
/// 
/// Header:
///  Byte 0         : number of structs
///  Byte 1         : 8 bit chksum for Byte 0
/// Struct 1:
///  Byte X         : N number of bytes for this struct
///  Byte X+1       : type_info
///  Byte X+2       : 8 bit chksum for Byte X : Byte X + 1
///  Byte X+3...    : data
/// Struct M :
///  Byte Y         : N number of bytes for this struct
///  Byte Y+1       : type_info
///  Byte Y+2       : 8 bit checksum for Byte Y : Byte Y + 1
///  Byte Y+3...    : data
/// CRC16
///  Byte Z         : 16 bit chksum # lower
///  Byte Z+1       : 16 bit chksum # higher

#ifndef PF_BOARD_COMMS_TRANSPORT_LAYER_H
#define PF_BOARD_COMMS_TRANSPORT_LAYER_H

#include <cstdint>

namespace pf_board
{
namespace comms
{


struct TypeInfo
{
  uint8_t* data_ptr=nullptr;
  uint8_t data_len=0;
  uint8_t type_enum=0;

  /// bool casting to check if TypeInfo is valid
  operator bool() const
  {
    return data_ptr != nullptr && data_len > 0 && type_enum != 0;
  }
};

class TransportLayer
{
 public:
  enum TypeEnum : uint8_t
  {
    /// each bit represents one type, not many types are expected
    invalid_type=0,
    control_type=1u,
    feedback_type=1u << 1,
    motor_cmd_request_type = 1u << 2,
    motor_cmd_response_type = 1u << 3
  };
  //// Functions for read operation

  /// Start Reading from target buffer
  /// @param source_buffer source location to begin reading data
  /// @param max_buffer_size maximum size of the buffer
  /// @param allowed_types white list of allowed TypeEnums, fails to read if any does not match
  /// @returns TypeInfo of first struct at index 0 if successful, otherwise returns an invalid TypeInfo
  TypeInfo beginRead(uint8_t* source_buffer, size_t max_buffer_size=256, uint8_t allowed_types=0xff);

  /// Retrieve current struct typeinfo
  TypeInfo currentRead();

  /// Get typeinfo of next struct, does not move the read position
  TypeInfo peekRead();

  /// Get typeinfo of the next struct, move the read position
  TypeInfo nextRead();


  /// Functions for WRITE operation
  /// usage:
  ///  beginWrite(...) or resetWrite() -> allocateWrite(..) or allocateWriteFrom(...) -> allocateWrite(..) or allocateWriteFrom(...) -> ... -> finalizeWrite()

  /// Start Writing to target buffer
  /// @param target_buffer target location to begin writing data
  /// @param max_buffer_size maximum size of the buffer
  void beginWrite(uint8_t* target_buffer, size_t max_buffer_size=256);

  /// Resets to the initial write state
  /// This reuses previous write buffer pointer
  void resetWrite();

  /// Allocate N bytes of data to be written
  /// @param len number of bytes to allocate
  /// @param type_enum see TypeEnum field
  /// @returns pointer to the first allocated byte, returns nullptr if failed to allocate
  uint8_t* allocateWrite(uint8_t len, TypeEnum type_enum=0);

  /// Allocate N bytes of data to be written, and copy from a source buffer
  /// @param source_data pointer to source data to copy from
  /// @param len number of bytes to allocate
  /// @param type_enum see TypeEnum field
  /// @returns true if successful, false otherwise
  bool allocateWriteFrom(uint8_t* source_data, uint8_t len, TypeEnum type_enum=0);

  /// Finalize all data, calculate number 
  /// @returns total number of written bytes including headers and checksum
  size_t finalizeWrite();

 private:
  bool readable_ = false;
  bool writable_ = false;
  uint8_t struct_index_ = 0;
  size_t byte_index_ = 0;
  size_t max_struct_index_ = 0;
  uint8_t* buf_ = nullptr;
  size_t max_buffer_size_ = 0;
};


}  // namespace transport_layer
}  // namespace pf_board

#endif  // PF_BOARD_COMMS_TRANSPORT_LAYER_H
