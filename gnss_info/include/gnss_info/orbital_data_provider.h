// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <cras_cpp_common/expected.hpp>
#include <geographic_msgs/GeoPoint.h>
#include <gnss_info_msgs/SatelliteInfo.h>
#include <gnss_info_msgs/SatellitePosition.h>
#include <gnss_info_msgs/SatelliteSkyPosition.h>

#include <ros/time.h>

namespace gnss_info
{

/**
 * \brief Generic interface for various providers of GNSS satellite orbital data.
 */
class OrbitalDataProvider
{
public:
    /**
     * \brief Constructing the provider should be a cheap and fast operation, ideally without internet access.
     */
    OrbitalDataProvider() = default;
    virtual ~OrbitalDataProvider() = default;

    virtual std::string getName() const = 0;
    virtual bool isPrecise() const = 0;
    virtual bool isApproximate() const = 0;

    virtual std::pair<ros::Time, ros::Time> getTimeRange() const = 0;
    virtual std::unordered_set<std::string> getConstellations() const = 0;

    virtual bool preload(const ros::Time& startTime, const ros::Time& endTime) = 0;

    virtual cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>, std::string>
    getECEFPositions(const ros::Time& time,
        const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satellites) = 0;

    virtual cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition>, std::string> getSkyView(
        const ros::Time& time, const geographic_msgs::GeoPoint& receiverPosition, double elevationMaskDeg,
        const std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>& satelliteECEFPositions) = 0;
};

}
