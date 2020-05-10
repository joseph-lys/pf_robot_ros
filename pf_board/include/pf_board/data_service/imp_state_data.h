/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef DATA_FROM_BOARD_H
#define DATA_FROM_BOARD_H


#include "fsm/base_classes.h"
#include "fsm/configuration.h"


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

struct ContextData : public pf_board::fsm::StateData
{
  std::map<uint8_t, uint16_t> motor_id_mapping;
  std::map<uint8_t, MotorFeedbackData> motor_feedback_data;
};

#endif  // DATA_FROM_BOARD_H
