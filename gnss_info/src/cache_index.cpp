// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <ctime>
#include <functional>

#include <gnss_info/cache_index.h>
#include <ros/time.h>

namespace gnss_info
{

DayIndex::DayIndex(const ros::Time& time)
{
    const time_t nowSec = time.sec;
    const auto t = *std::gmtime(&nowSec);
    this->year = t.tm_year + 1900;  // struct tm year is relative to year 1900
    this->month = t.tm_mon + 1;
    this->day = t.tm_mday;
}

DayIndex::DayIndex(const uint16_t year, const uint8_t month, const uint8_t day):
    year(year), month(month), day(day)
{
}

DayIndex::operator ros::Time() const
{
    tm start{};
    start.tm_year = this->year - 1900;
    start.tm_mon = this->month - 1;
    start.tm_mday = this->day;
    return ros::Time(std::mktime(&start), 0);
}

bool DayIndex::operator==(const DayIndex& other) const
{
    return this->year == other.year && this->month == other.month && this->day == other.day;
}

}

std::size_t std::hash<gnss_info::DayIndex>::operator()(const gnss_info::DayIndex& k) const noexcept
{
    return ((std::hash<uint16_t>()(k.year) ^ (std::hash<uint8_t>()(k.month) << 1)) >> 1)
        ^ (std::hash<uint8_t>()(k.day) << 1);
}
