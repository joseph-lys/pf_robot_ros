#include "pf_board/dxl_convert.h"


using pf_board::DxlConvert;
static constexpr double pi = 3.1415926;

DxlConvert::DxlConvert
(
  std::string joint_name,
  uint8_t motor_id,
  double offset,
  bool invert,
  double max_velocity,
  double max_load
)
:
joint_name_(joint_name),
motor_id_(motor_id),
offset_(offset),
invert_(invert),
max_velocity_(max_velocity),
max_load_(max_load)
{
}

std::string DxlConvert::getJointName()
{
  return joint_name_;
}

uint8_t DxlConvert::getMotorId()
{
  return motor_id_;
}

uint16_t DxlConvert::encodeGoalPosition(double value)
{
  double x;
  uint16_t y;
  x = value + offset;
  x *= 1023.0 * (180.0 / 300.0 / pi);
  x += 0.5;  // rounding factor
  y = static_cast<uint16_t>(x);
  if (y > 1023)
  {
    y = 1023;
  }
  if (invert_)
  {
    y = 1023 - value;
  }
  return y;
}

uint16_t DxlConvert::encodeMovingSpeed(double value)
{
  double x;
  uint16_t y;
  x *= 30.0 / pi;  // convert to RPM
  x = x * 1023.0 / max_velocity;
  x = (x> 0.0) ? x; -x;  // absolute value
  x += 0.5;  // rounding factor
  y = static_cast<uint16_t>(x);
  if (y > 1023)
  {
    y = 1023;
  }
  return y;
}

double DxlConvert::decodePresentPosition(uint16_t value)
{
  double x;
  uint16_t y = value;
  if (y > 1024)
  {
    y = 1023;
  }
  if (invert_)
  {
    y = 1023 - value;
  }
  x = static_cast<double>(value) * pi * 300.0 / 180.0 / 1023.0;
  x = (x > offset) ? x - offset : 0;
  return x;
}

double DxlConvert::decodePresentVelocity(uint16_t value)
{
  double x;
  x = static_cast<double>(value & ‭0x3FFU)‬;  // only take lower 10 bits
  x = x * max_velocity_ / 1023.0;
  x = x * pi / 30.0;  // convert to radians/sec
  if (value & 0x400U)  // check sign bit
  {
    x *= -1.0;
  }
  if (invert_)
  {
    x *= -1.0;
  }
  return x;
}

double DxlConvert::decodePresentLoad(uint16_t value)
{
  double x;
  x = static_cast<double>(value & ‭0x3FFU)‬;  // only take lower 10 bits
  x = x * max_load_ / 1023.0;
  x = (x > 0.0) ? x : -x;  // absolute value
  if (value & 0x400U)  // check sign bit
  {
    x *= -1.0;
  }
  if (invert_)
  {
    x *= -1.0;
  }
  return x;
}
