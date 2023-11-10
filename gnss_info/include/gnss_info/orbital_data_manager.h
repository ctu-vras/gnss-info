// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <cras_cpp_common/expected.hpp>
#include <geographic_msgs/GeoPoint.h>
#include <gnss_info/orbital_data_provider.h>
#include <gnss_info_msgs/DOP.h>
#include <gnss_info_msgs/SatellitesList.h>
#include <gnss_info_msgs/SatellitesPositions.h>
#include <gnss_info_msgs/SkyView.h>

#include <ros/time.h>

namespace gnss_info
{

struct OrbitalDataManagerPrivate;

class OrbitalDataManager
{
public:
    OrbitalDataManager();
    virtual ~OrbitalDataManager();

    void addProvider(const std::shared_ptr<OrbitalDataProvider>& provider);

    bool preload(const ros::Time& time);
    bool preload(const ros::Time& time, const cras::optional<bool>& precise);
    bool preload(const ros::Time& startTime, const ros::Time& endTime);
    virtual bool preload(const ros::Time& startTime, const ros::Time& endTime, const cras::optional<bool>& precise);

    cras::expected<gnss_info_msgs::SatellitesPositions, std::string> getPositions(
        const ros::Time& time, const gnss_info_msgs::SatellitesList& satellites);

    virtual cras::expected<gnss_info_msgs::SatellitesPositions, std::string> getPositions(
        const ros::Time& time, const gnss_info_msgs::SatellitesList& satellites, const cras::optional<bool>& precise);

    cras::expected<gnss_info_msgs::SkyView, std::string> getSkyView(
        const geographic_msgs::GeoPoint& position, const gnss_info_msgs::SatellitesPositions& positions,
        double elevationMaskDeg);

    virtual cras::expected<gnss_info_msgs::SkyView, std::string> getSkyView(
        const geographic_msgs::GeoPoint& position, const gnss_info_msgs::SatellitesPositions& positions,
        double elevationMaskDeg, const cras::optional<bool>& precise);

    virtual gnss_info_msgs::DOP computeDOP(
        const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition>& skyView) const;

protected:
    std::unique_ptr<OrbitalDataManagerPrivate> data;  //!< Private implementation details (PIMPL).
};

}
