/// Copyright 2020 Joseph Lee Yuan Sheng
///

#ifndef PF_BOARD_H
#define PF_BOARD_H

#include <string>
#include "ros/ros.h"

namespace pf_board
{

class PfBoard
{
 public:
  explicit PfBoard(ros::NodeHandle& nodeHandle);
  ~PfBoard();

 private:
  bool readParameters();
  ros::NodeHandle& nodeHandle_;
  std::string subscriber_topic_;
};


}  // namespace pf_board


#endif  // PF_BOARD_H
