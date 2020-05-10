#include "pf_board/simple_timer.h"

SimpleTimer::SimpleTimer()
: duration_ns_(0)
{
  setMillis(0);
}

bool SimpleTimer::isTimeout()
{
  return Time::now().toNSec() - starting_time_ns_ < duration_ns_;
}

void SimpleTimer::reset()
{
  starting_time_ns_ = Time::now().toNSec();
}

void SimpleTimer::setMillis(uint32_t ms)
{
  duration_ns_ = static_cast<uint64_t>(ms) * 1000000ull;
  reset();
}