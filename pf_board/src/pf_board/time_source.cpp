#include "pf_board/time_source.h"


using pf_board::TimeSource;


ros::Time TimeSource::now()
{
  return ros::Time::now();
}

void TimeSource::setNow(const ros::Time& new_now)
{
  ros::Time::setNow(new_now);
}

void TimeSource::setStamp()
{
  stamp_ = now();
}

ros::Time TimeSource::getStamp()
{
  return stamp_;
}