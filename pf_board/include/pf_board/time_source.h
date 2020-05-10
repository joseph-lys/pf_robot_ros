/// Copyright 2020 Joseph Lee Yuan Sheng
///
/// time source singleton
/// provides an adapter ros time implementation
/// so that underlying implementation can be swapped to a mock time source for testing

#define PF_BOARD_TIME_SOURCE_H

#include <vector>
#include "ros/ros.h"

namespace pf_board
{
class TimeSource
{
 public:
  static ros::Time now();
  static void setNow(const Time& new_now);
  static void setStamp();  // store a timestamp that can be reused later
  static ros::Time getStamp(); // retrieve the last timestamp

  Timesource() = delete;
 private:
  static ros::Time stamp_{0};
}; 


}  // namespace pf_board
#endif  // PF_BOARD_TIME_SOURCE_H