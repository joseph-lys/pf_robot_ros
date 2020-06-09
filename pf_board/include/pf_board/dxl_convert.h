/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_H
#define PF_BOARD_H


#include <string>

namespace pf_board
{


class DxlConvert
{
  DxlConvert() = delete;
  ~DxlConvert() = default;

  /// Constructor
  /// @param joint_name
  /// @param motor_id
  /// @param offset correction in radians
  /// @param invert direction inversion, inverted if true
  /// @param max_velocity veloctiy at full value (rad/s)
  /// @param max_load load at full value (N/m)
  explicit DxlConvert
  (
    std::string joint_name,
    uint8_t motor_id,
    double offset,
    bool invert,
    double max_velocity,
    double max_load
  );

  std::string getJointName();

  uint8_t getMotorId();

  /// Target goal position in radians
  uint16_t encodeGoalPosition(double);
  
  /// Target moving speed in radians/s
  uint16_t encodeMovingSpeed(double);

  /// Feedback Data

  /// Current position in radians
  double decodePresentPosition(uint16_t);

  /// Current speed in radians/s
  double decodePresentVelocity(uint16_t);
  
  /// Current load in N/m
  double decodePresentLoad(uint16_t);

  private:
  const std::string joint_name_;
  const uint8_t motor_id_;
  const bool invert_;
  const double offset_;
  const double max_velocity_;
  const double max_load_;
};


}  // namespace pf_board

#endif  // PF_BOARD_H

