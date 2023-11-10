// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <cras_cpp_common/expected.hpp>
#include <gnss_info/orbital_data_provider.h>
#include <gnss_info_msgs/SatelliteInfo.h>
#include <gnss_info_msgs/SatellitePosition.h>
#include <gnss_info_msgs/SatelliteSkyPosition.h>

#include <ros/time.h>

namespace gnss_info
{

struct EthzSatdbProviderPrivate;

/**
 * \brief Generic interface for various providers of GNSS satellite orbital data.
 */
class EthzSatdbProvider : public OrbitalDataProvider
{
public:
    explicit EthzSatdbProvider(const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satelliteInfo);
    ~EthzSatdbProvider() override;

    std::string getName() const override;
    bool isPrecise() const override;
    bool isApproximate() const override;

    std::pair<ros::Time, ros::Time> getTimeRange() const override;
    std::unordered_set<std::string> getConstellations() const override;

    bool preload(const ros::Time& startTime, const ros::Time& endTime) override;

    cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>, std::string> getECEFPositions(
        const ros::Time& time, const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satellites) override;

    cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition>, std::string> getSkyView(
        const ros::Time& time, const geographic_msgs::GeoPoint& receiverPosition, double elevationMaskDeg,
        const std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>& satelliteECEFPositions) override;

private:
    std::unique_ptr<EthzSatdbProviderPrivate> data;  //!< Private implementation data (PIMPL).
};

}
