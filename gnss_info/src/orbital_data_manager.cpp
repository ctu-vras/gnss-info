// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>

#include <Eigen/Eigen>

#include <angles/angles.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <gnss_info/orbital_data_manager.h>
#include <gnss_info/orbital_data_provider.h>
#include <gnss_info_msgs/SatellitesList.h>
#include <gnss_info_msgs/SatellitesPositions.h>
#include <gnss_info_msgs/SkyView.h>
#include <ros/ros.h>

namespace gnss_info
{

struct OrbitalDataManagerPrivate
{
    std::list<std::shared_ptr<OrbitalDataProvider>> providers;

    std::list<std::shared_ptr<OrbitalDataProvider>> getRelevantProviders(
        const ros::Time& time, const cras::optional<bool>& precise) const;

    std::list<std::shared_ptr<OrbitalDataProvider>> getRelevantProviders(
        const ros::Time& startTime, const ros::Time& endTime, const cras::optional<bool>& precise) const;
};

std::list<std::shared_ptr<OrbitalDataProvider>> OrbitalDataManagerPrivate::getRelevantProviders(
    const ros::Time& time, const cras::optional<bool>& precise) const
{
    return this->getRelevantProviders(time, time + ros::Duration::MINUTE, precise);
}

std::list<std::shared_ptr<OrbitalDataProvider>> OrbitalDataManagerPrivate::getRelevantProviders(
    const ros::Time& startTime, const ros::Time& endTime, const cras::optional<bool>& precise) const
{
    std::list<std::shared_ptr<OrbitalDataProvider>> result;
    for (const auto& provider : this->providers)
    {
        if (precise.has_value() && precise && !provider->isPrecise())
            continue;
        if (precise.has_value() && !precise && !provider->isApproximate())
            continue;
        if (startTime > provider->getTimeRange().second)
            continue;
        if (endTime < provider->getTimeRange().first)
            continue;
        result.push_back(provider);
    }
    return result;
}

OrbitalDataManager::OrbitalDataManager() : data(new OrbitalDataManagerPrivate)
{
}

OrbitalDataManager::~OrbitalDataManager() = default;

bool OrbitalDataManager::preload(const ros::Time& time)
{
    return this->preload(time, time + ros::Duration::MINUTE, cras::nullopt);
}

bool OrbitalDataManager::preload(const ros::Time& time, const cras::optional<bool>& precise)
{
    return this->preload(time, time + ros::Duration::MINUTE, precise);
}

bool OrbitalDataManager::preload(const ros::Time& startTime, const ros::Time& endTime)
{
    return this->preload(startTime, endTime, cras::nullopt);
}

bool OrbitalDataManager::preload(
    const ros::Time& startTime, const ros::Time& endTime, const cras::optional<bool>& precise)
{
    for (const auto& provider : this->data->getRelevantProviders(startTime, endTime, precise))
    {
        if (provider->preload(startTime, endTime))
            return true;
    }
    return false;
}

cras::expected<gnss_info_msgs::SatellitesPositions, std::string> OrbitalDataManager::getPositions(
    const ros::Time& time, const gnss_info_msgs::SatellitesList& satellites)
{
    return this->getPositions(time, satellites, cras::nullopt);
}

cras::expected<gnss_info_msgs::SatellitesPositions, std::string> OrbitalDataManager::getPositions(
    const ros::Time& time, const gnss_info_msgs::SatellitesList& satellites, const cras::optional<bool>& precise)
{
    this->preload(time);
    const auto providers = this->data->getRelevantProviders(time, precise);
    if (providers.empty())
        return cras::make_unexpected("No satellite data was found");

    std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo> sats;
    for (const auto& sat : satellites.satellites)
        sats[sat.satcat_id] = sat;

    for (const auto& provider : providers)
    {
        const auto maybePositions = provider->getECEFPositions(time, sats);
        if (!maybePositions.has_value())
        {
            ROS_DEBUG("Failed getting positions from provider %s at time %s: %s",
                      provider->getName().c_str(), cras::to_string(time).c_str(), maybePositions.error().c_str());
            continue;
        }

        const auto& positions = *maybePositions;
        if (positions.empty())
            continue;

        gnss_info_msgs::SatellitesPositions msg;
        msg.header.stamp = time;
        msg.header.frame_id = "ecef";
        for (const auto& [satcatId, pos] : positions)
            msg.satellites.push_back(pos);
        return msg;
    }
    return cras::make_unexpected("Could not find a provider for satellite ECEF positions.");
}

cras::expected<gnss_info_msgs::SkyView, std::string> OrbitalDataManager::getSkyView(
    const geographic_msgs::GeoPoint& position, const gnss_info_msgs::SatellitesPositions& positions,
    const double elevationMaskDeg)
{
    return this->getSkyView(position, positions, elevationMaskDeg, cras::nullopt);
}

cras::expected<gnss_info_msgs::SkyView, std::string> OrbitalDataManager::getSkyView(
    const geographic_msgs::GeoPoint& position, const gnss_info_msgs::SatellitesPositions& positions,
    const double elevationMaskDeg, const cras::optional<bool>& precise)
{
    this->preload(positions.header.stamp);
    const auto providers = this->data->getRelevantProviders(positions.header.stamp, precise);
    if (providers.empty())
        return cras::make_unexpected("No satellite data was found");

    std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition> positionsMap;
    for (const auto& pos : positions.satellites)
        positionsMap[pos.satcat_id] = pos;

    for (const auto& provider : providers)
    {
        const auto maybeSkyView = provider->getSkyView(
            positions.header.stamp, position, elevationMaskDeg, positionsMap);

        if (!maybeSkyView.has_value())
        {
            ROS_DEBUG("Failed getting sky view from provider %s at time %s: %s",
                      provider->getName().c_str(), cras::to_string(time).c_str(), maybeSkyView.error().c_str());
            continue;
        }

        const auto& skyView = *maybeSkyView;
        gnss_info_msgs::SkyView msg;
        msg.header.stamp = positions.header.stamp;
        msg.header.frame_id = "WGS84";
        msg.elevation_mask_deg = elevationMaskDeg;
        msg.reference_position = position;
        for (const auto& [satcatId, sat] : skyView)
            msg.satellites.push_back(sat);
        msg.dop = this->computeDOP(skyView);
        return msg;
    }

    return cras::make_unexpected("Could not find a provider for satellite sky positions.");
}

gnss_info_msgs::DOP OrbitalDataManager::computeDOP(
    const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition>& skyView) const
{
    // https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)#Computation
    // rtklib uses exactly the same algorithm in dops().

    gnss_info_msgs::DOP dop;

    if (skyView.size() < 4)
    {
        dop.gdop = dop.pdop = dop.hdop = dop.vdop = dop.tdop = std::numeric_limits<float>::infinity();
        return dop;
    }

    Eigen::MatrixX4d A(skyView.size(), 4);
    Eigen::Index i{0};
    for (const auto& [satcatId, skyPos] : skyView)
    {
        const auto az = angles::from_degrees(skyPos.azimuth_deg);
        const auto el = angles::from_degrees(skyPos.elevation_deg);
        A(i, 0) = std::cos(el) * std::sin(az);
        A(i, 1) = std::cos(el) * std::cos(az);
        A(i, 2) = std::sin(el);
        A(i, 3) = 1.0;
        ++i;
    }

    const Eigen::Matrix4d Q = (A.transpose() * A).inverse();
    dop.tdop = std::sqrt(Q(3, 3));
    dop.vdop = std::sqrt(Q(2, 2));
    dop.hdop = std::sqrt(Q(0, 0) + Q(1, 1));
    dop.pdop = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
    dop.gdop = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2) + Q(3, 3));

    return dop;
}

void OrbitalDataManager::addProvider(const std::shared_ptr<OrbitalDataProvider>& provider)
{
    this->data->providers.push_back(provider);
}

}
