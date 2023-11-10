// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <gnsstk/CommonTime.hpp>

#include <ros/time.h>

namespace gnsstk_ros
{

/**
 * \brief UNIX epoch (1 Jan 1970) represented in gnsstk CommonTime means. Static values for efficient computations.
 */
struct UnixEpoch
{
    static const gnsstk::CommonTime gnsstkCommonTime;  //!< CommonTime of the UNIX epoch.
    //! Number of days between CommonTime beginning of time and UNIX epoch start.
    static const long gnsstkDaysOffset;  // NOLINT(runtime/int)
};

/**
 * \brief Get the number of days stored in the given time object.
 * \param[in] time The time to get days of.
 * \return The number of days.
 */
long getDays(const gnsstk::CommonTime& time);  // NOLINT(runtime/int)

/**
 * \brief Convert the given gnsstk time to ROS time.
 * \param[in] commonTime The time to convert.
 * \return The equivalent ROS time.
 * \throws std::runtime_error If the time cannot be represented in ROS.
 */
ros::Time convert(const gnsstk::CommonTime& commonTime);

/**
 * \brief Covnert the given ROS time to gnsstk time.
 * \param[in] rosTime The ROS time to convert.
 * \return The equivalent gnsstk time.
 */
gnsstk::CommonTime convert(const ros::Time& rosTime);

}
