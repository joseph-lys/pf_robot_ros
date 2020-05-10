#include "time_source/time_source.h"



ros::Time TimeSource::now()
{
  return ros::Time::now();
}

void TimeSource::setNow(const Time& new_now)
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