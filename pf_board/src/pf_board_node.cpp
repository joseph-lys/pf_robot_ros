/// Copyright 2020 Joseph Lee Yuan Sheng
///

#include <ros/ros.h>
#include "./pf_board.h"
#include "fsm/context_data.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pf_board");
  ros::NodeHandle nodeHandle("~");
  pf_board::PfBoard PfBoard(nodeHandle);

  ros::spin();
  return 0;
}
