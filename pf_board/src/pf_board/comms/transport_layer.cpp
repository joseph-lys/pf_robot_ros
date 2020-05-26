/// Copyright 2020 Joseph Lee Yuan Sheng
///

#include "pf_board/comms/transport_layer.h"
#include <string.h>

using pf_board::comms::TransportLayer;
using pf_board::comms::TypeInfo;

static unsigned char const crc8x_table[] = {
    0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb, 0xea, 0x7d,
    0x4c, 0x1f, 0x2e, 0x43, 0x72, 0x21, 0x10, 0x87, 0xb6, 0xe5, 0xd4, 0xfa, 0xcb,
    0x98, 0xa9, 0x3e, 0x0f, 0x5c, 0x6d, 0x86, 0xb7, 0xe4, 0xd5, 0x42, 0x73, 0x20,
    0x11, 0x3f, 0x0e, 0x5d, 0x6c, 0xfb, 0xca, 0x99, 0xa8, 0xc5, 0xf4, 0xa7, 0x96,
    0x01, 0x30, 0x63, 0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb, 0x3d,
    0x0c, 0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa, 0x84, 0xb5, 0xe6, 0xd7, 0x40, 0x71,
    0x22, 0x13, 0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9, 0xc7, 0xf6, 0xa5,
    0x94, 0x03, 0x32, 0x61, 0x50, 0xbb, 0x8a, 0xd9, 0xe8, 0x7f, 0x4e, 0x1d, 0x2c,
    0x02, 0x33, 0x60, 0x51, 0xc6, 0xf7, 0xa4, 0x95, 0xf8, 0xc9, 0x9a, 0xab, 0x3c,
    0x0d, 0x5e, 0x6f, 0x41, 0x70, 0x23, 0x12, 0x85, 0xb4, 0xe7, 0xd6, 0x7a, 0x4b,
    0x18, 0x29, 0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x07, 0x36, 0x65,
    0x54, 0x39, 0x08, 0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1, 0xe2, 0xd3,
    0x44, 0x75, 0x26, 0x17, 0xfc, 0xcd, 0x9e, 0xaf, 0x38, 0x09, 0x5a, 0x6b, 0x45,
    0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2, 0xbf, 0x8e, 0xdd, 0xec, 0x7b, 0x4a,
    0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xc2, 0xf3, 0xa0, 0x91, 0x47, 0x76, 0x25,
    0x14, 0x83, 0xb2, 0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0x0b, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93, 0xbd, 0x8c, 0xdf, 0xee, 0x79,
    0x48, 0x1b, 0x2a, 0xc1, 0xf0, 0xa3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49,
    0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef, 0x82, 0xb3, 0xe0, 0xd1, 0x46, 0x77, 0x24,
    0x15, 0x3b, 0x0a, 0x59, 0x68, 0xff, 0xce, 0x9d, 0xac};

static uint8_t calculateCRC8(uint8_t* buffer, size_t length)
{
  /// https://stackoverflow.com/questions/51752284/how-to-calculate-crc8-in-c
  uint8_t crc = 0xff;
  if (buffer == nullptr)
      return 0xff;
  crc &= 0xff;
  while (length--)
      crc = crc8x_table[crc ^ *buffer++];
  return crc;
}

static uint16_t calculateCRC16(uint8_t* buffer, size_t length)
{
  /// Todo: just return some value for now
  return 0xAABB;
}


TypeInfo TransportLayer::beginRead(uint8_t* buffer_source, size_t max_buffer_size, uint8_t allowed_types)
{
  byte_index_ = 0;
  struct_index_ = 0;
  max_struct_index_ = 0;
  writable_ = false;
  readable_ = false;
  uint8_t x, crc8, temp_struct_index, temp_max_struct_index;
  uint16_t y, crc16;
  size_t temp_byte_index;
  TypeInfo t{};

  if (buffer_source == nullptr)
  {
    // error, invalid source!
  }
  else if (buffer_source[0] < 1)
  {
    // error, no structs!
  }
  else if (calculateCRC8(&buffer_source[0], 1) != buffer_source[1])
  {
    // error, size crc mismatch!
  }
  else
  {
    temp_max_struct_index = buffer_source[0];
    temp_struct_index = 0;
    temp_byte_index = 2ULL;
    bool success = true;
    for (temp_struct_index = 0; temp_struct_index<temp_max_struct_index; temp_max_struct_index++)
    {
      x = buffer_source[temp_byte_index];
      if (temp_byte_index + 3ULL + static_cast<size_t>(x) > max_buffer_size)
      {
        /// out of bound condition
        success = false;
        break;
      }
      if (buffer_source[temp_byte_index + 1ULL] & allowed_types == 0)
      {
        // not an accepted type!
        success = false;
        break;
      }
      crc8 = calculateCRC8(&buffer_source[temp_byte_index], 3ULL);
      if (crc8 != buffer_source[temp_byte_index + 2ULL])
      {
        // error in the size!
        success = false;
        break;
      }
      // move the byte position of the next struct.
      // if this is the last struct, the updated index will represent the length
      temp_byte_index += 3ULL + static_cast<size_t>(x);
    }
    if (temp_byte_index + 2ULL> max_buffer_size))
    {
      // not possible, crc16 is out of bound!
    }
    else if (success)
    {
      crc16 = calculateCRC16(&buffer_source[0], temp_byte_index);
      y = static_cast<uint16_t>(buffer_source[temp_byte_index]) & 0xff;
      y |= static_cast<uint16_t>(buffer_source[temp_byte_index + 1]) << 8;
      if (y == crc16)
      {
        // all ok, can initialize the read data
        max_struct_index_ = temp_max_struct_index;
        byte_index_ = 2ULL;
        struct_index_ = 0U;
        buf_ = buffer_source;
        readable_ = true;

        t = currentRead();
      }
    }
  }
  return t;
}

TypeInfo TransportLayer::currentRead()
{
  TypeInfo t{};
  if (readable_)
  {
    t.data_ptr = &buf_[byte_index_ + 3];
    t.data_len = buf_[byte_index_];
    t.type_enum = buf_[byte_index_ + 1];
  }
  return t;
}

TypeInfo TransportLayer::peekRead()
{
  TypeInfo t{};
  if (!readable_)
  {
    // error, check if previous call to beginRead(...) was successful!
  }
  else if (struct_index_ + 1 < max_struct_index_)
  {
    size_t temp_index = byte_index_ + 3ULL + static_cast<size_t>(buf_[byte_index_]);
    t.data_ptr = &buf_[temp_index + 3];
    t.data_len = buf_[temp_index];
    t.type_enum = buf_[temp_index + 1];
  }
  return t;
}

TypeInfo TransportLayer::nextRead()
{
  TypeInfo t = peek;
  if (t)
  {
    byte_index_ += 3ULL + static_cast<size_t>(buf_[byte_index_]);
  }
  return t;
}

void TransportLayer::beginWrite(uint8_t* target_buffer, size_t max_buffer_size)
{
  buf_ = target_buffer;
  max_buffer_size_ = max_buffer_size;
  clearWrite();
}

void TransportLayer::clearWrite()
{
  readable_ = false;
  writable_ = false;

  if (buf_ != nullptr && max_buffer_size_ > 7)
  {
    byte_index_ = 0;  // start at 0, this indicates no allocations
    struct_index_ = 0;
    writable_ = true;
  }
}

uint8_t* TransportLayer::allocateWrite(uint8_t len, TypeEnum type_enum=0)
{
  uint8_t* ptr = nullptr;
  bool can_allocate = false;
  if (buf_ == nullptr)
  {
    // error, out of sequence call, should call beginWrite(...) first!
  }
  else if (!writable_)
  {
    // error, check if previous call to beginWrite(...) was successful!
  }
  else if (type_enum == 0)
  {
    // error, this is not a valid type!
  }
  else if (byte_index_ == 0) // first allocation!
  {
    // 7 bytes reserved for number of structs (2), struct header (3), and crc16(2)
    if (byte_index_ + 7ULL + static_cast<size_t>(len) <= max_buffer_size_)
    {
      struct_index_ = 0;
      byte_index_ = 2ULL;
      ptr = &buf_[byte_index_ + 3];
      buf_[byte_index_] = len;
      buf_[byte_index_ + 1] = static_cast<uint8_t>(type_enum);
      buf_[byte_index_ + 2] = calculateCRC8(&buf_[byte_index_], 2);
    }
  }

  else if (byte_index_ > 0 && current_)  // already allocated, start on next 
  {
    // 5 bytes reserved for struct header (3), and crc16(2)
    if (byte_index_ + buf_[byte_index_] + 5ULL + static_cast<size_t>(len) <= max_buffer_size_)
    {
      struct_index_++;
      byte_index_ += 3ULL + static_cast<size_t>(buf_[byte_index_]);
      ptr = &buf_[byte_index_ + 3];
      buf_[byte_index_] = len;
      buf_[byte_index_ + 1] = static_cast<uint8_t>(type_enum);
      buf_[byte_index_ + 2] = calculateCRC8(&buf_[byte_index_], 2);
    }
  }
  return ptr;
}

bool TransportLayer::allocateWriteFrom(uint8_t* source_data, uint8_t len, TypeEnum type_enum)
{
  bool success = false;
  uint8_t* ptr = allocateWrite();  // all checking is already done in allocateWrite(...)
  if (ptr != nullptr)
  {
    memcpy(
      reinterpret_cast<void*>(ptr),          // destination
      reinterpret_cast<void*>(source_data),  // source
      static_cast<size_t>(len)               // num
    );
    success = true;
  }
  return success;
}

size_t finalizeWrite()
{
  size_t total_bytes = 0;
  uint16_t crc16;
  if (buf_ == nullptr)
  {
    // error, out of sequence call, should call beginWrite(...) first!
  }
  else if (!writable_)
  {
    // error, check if previous call to beginWrite(...) was successful!
  }
  else if (byte_index_ == 0) // first allocation!
  {
    // error, finalized empty data!
  }
  else
  {
    // finalize the header
    buf_[0] = 1 + static_cast<uint8_t>(struct_index_);  // finalize total number of structs
    buf_[1] = calculateCRC8(&buf_[0], 1);

    // increment the byte index, the byte_index_ now points to the CRC16;
    byte_index_ += 3ULL + static_cast<size_t>(buf_[byte_index_]);
    crc16 = calculateCRC16(&buf_[0], byte_index_);
    buf_[byte_index_] = static_cast<uint8_t>(crc16 & 0xff);
    buf_[byte_index_ + 1] = static_cast<uint8_t>(crc16 >> 8);
    
    byte_index_ += 2ULL;  // represent the total number of bytes including crc
    writable_ = false;
    total_bytes = byte_index_;
  }
  return total_bytes;
}