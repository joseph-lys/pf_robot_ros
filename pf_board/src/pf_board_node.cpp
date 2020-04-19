/// Copyright 2020 Joseph Lee Yuan Sheng
///

#include <ros/ros.h>
#include "./pf_board.h"
#include "fsm/defined_states.h"


int main(int argc, char** argv)
{
  auto x = pf_board::fsm::states::PreloadState::entry();
  ros::init(argc, argv, "pf_board");
  ros::NodeHandle nodeHandle("~");
  pf_board::PfBoard PfBoard(nodeHandle);

  ros::spin();
  return 0;
}
