// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <functional>

#include <ros/time.h>

namespace gnss_info
{

/**
 * \brief Object that can be used as an index into a cache and assigns all times from the same day the same cache index.
 */
struct DayIndex
{
    uint16_t year;  //!< Year.
    uint8_t month;  //!< Month (starting with 1).
    uint8_t day;  //!< Day (starting with 1).

    /**
     * \brief Create an index entry for the given time.
     * \param[in] time The time to create the index entry for.
     */
    explicit DayIndex(const ros::Time& time);

    /**
     * \brief Directly create an index entry for the given day.
     * \param[in] year The year to represent.
     * \param[in] month The month to represent (starting with 1).
     * \param[in] day The day to represent (starting with 1).
     */
    DayIndex(uint16_t year, uint8_t month, uint8_t day);

    /**
     * \brief Convert the index to a representative ROS Time.
     */
    explicit operator ros::Time() const;

    /**
     * \brief Compare with another index.
     * \param[in] other The other index to compare.
     * \return Whether this index represents the same day.
     */
    bool operator==(const DayIndex& other) const;
};

}

/**
 * \brief Hashing support for DayIndex.
 */
template<> struct std::hash<gnss_info::DayIndex>
{
    std::size_t operator()(const gnss_info::DayIndex& k) const noexcept;
};
