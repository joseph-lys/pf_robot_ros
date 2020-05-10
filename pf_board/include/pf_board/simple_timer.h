/// Copyright 2020 Joseph Lee Yuan Sheng
///
/// time source singleton
/// provides an adapter ros time implementation
/// so that underlying implementation can be swapped to a mock time source for testing

#ifndef PF_BOARD_SIMPLE_TIMER_H
#define PF_BOARD_SIMPLE_TIMER_H

#include <vector>
#include "pf_board/time_source.h"


namespace pf_board
{


class SimpleTimer
{
 public:
  SimpleTimer();
  bool isTimeout();
  void reset();
  void setMillis(uint32_t);
 private:
  uint64_t starting_time_ns_=0;
  uint64_t duration_ns_=0;
};


} // namespace pf_board

#endif  // PF_BOARD_SIMPLE_TIMER_H
