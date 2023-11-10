// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <limits>

#include <gnsstk/CommonTime.hpp>
#include <gnsstk/YDSTime.hpp>

#include <gnsstk_ros/time.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace gnsstk_ros
{

// Statically initialize the number of days since 1 Jan 1970 in gnsstk::CommonTime.
const gnsstk::CommonTime UnixEpoch::gnsstkCommonTime {gnsstk::YDSTime(1970, 0, 0.0).convertToCommonTime()};
const long UnixEpoch::gnsstkDaysOffset {getDays(UnixEpoch::gnsstkCommonTime)};  // NOLINT(runtime/int)

long getDays(const gnsstk::CommonTime& time)  // NOLINT(runtime/int)
{
    long days, secs;  // NOLINT(runtime/int)
    double frac;
    time.get(days, secs, frac);
    return days;
}

ros::Time convert(const gnsstk::CommonTime& commonTime)
{
    long days, secs;  // NOLINT(runtime/int)
    double frac;
    commonTime.get(days, secs, frac);

    const auto daysSecs = ros::Duration::DAY.sec * (days - UnixEpoch::gnsstkDaysOffset);
    const auto timeSecs = daysSecs + secs;

    using SecType = decltype(ros::Duration::sec);
    if (timeSecs < std::numeric_limits<SecType>::min() || timeSecs > std::numeric_limits<SecType>::max())
        throw std::runtime_error("Duration is out of dual 32-bit range");

    return ros::Time(static_cast<SecType>(timeSecs), 0) + ros::Duration(frac);
}

gnsstk::CommonTime convert(const ros::Time& rosTime)
{
    auto commonTime = UnixEpoch::gnsstkCommonTime;
    commonTime.addDays(rosTime.sec / ros::Duration::DAY.sec);
    commonTime.addSeconds(static_cast<long>(rosTime.sec % ros::Duration::DAY.sec));  // NOLINT(runtime/int)
    commonTime.addSeconds(rosTime.nsec * 1e-9);
    return commonTime;
}

}
