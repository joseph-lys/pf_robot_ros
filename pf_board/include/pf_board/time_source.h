/// Copyright 2020 Joseph Lee Yuan Sheng
///
/// time source singleton
/// provides an adapter ros time implementation
/// so that underlying implementation can be swapped to a mock time source for testing
#ifndef PF_BOARD_TIME_SOURCE_H
#define PF_BOARD_TIME_SOURCE_H

#include <vector>
#include "ros/ros.h"

namespace pf_board
{
class TimeSource
{
 public:
  static ros::Time now();
  static void setNow(const ros::Time& new_now);
  static void setStamp();  // store a timestamp that can be reused later
  static ros::Time getStamp(); // retrieve the last timestamp

 private:
  static ros::Time stamp_;
}; 


}  // namespace pf_board
#endif  // PF_BOARD_TIME_SOURCE_H