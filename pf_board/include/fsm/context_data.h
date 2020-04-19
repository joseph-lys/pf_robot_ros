/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef DATA_FROM_BOARD_H
#define DATA_FROM_BOARD_H


#include "fsm/base_classes.h"
#include "pf_msgs/MotorStateArray.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

enum ConfigurationConstants: uint8_t 
{
  kMaxMotors = 32,
  kMaxAI = 8,
  kMaxDI = 2,
  kMaxDO = 1,
  kMaxIMUs = 2
};

struct DataFromBoard
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

struct DataToBoard
{ 
  // Motor
  uint16_t position[kMaxMotors];
  uint16_t speed[kMaxMotors];
  bool enabled[kMaxMotors];

  // I/O
  bool digital_out[kMaxDO];
};

struct MotorFeedbackData {
  enum EnumConstants : int
  {
    kMaxReadings = 5
  };
  enum EnumOverallState : int
  {
    kStatusError = -2,
    kCommError = -1,
    kUnknown = 0,
    kValid = 1
  };
  EnumOverallState overall_state = EnumOverallState::kUnknown;
  uint8_t error = 0;
  int readings = 0;
  std::array<double, kMaxReadings> position_readings;
  std::array<double, kMaxReadings> speed_readings;
  std::array<double, kMaxReadings> torque_readings;
  unsigned long long last_reading_us = 0;
};

struct ContextData : public pf_board::fsm::BaseContext
{
  std::map<uint8_t, uint16_t> motor_id_mapping;
  std::map<uint8_t, MotorFeedbackData> motor_feedback_data;
};

#endif  // DATA_FROM_BOARD_H
