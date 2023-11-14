// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <algorithm>
#include <ctime>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gnsstk/MultiFormatNavDataFactory.hpp>
#include <gnsstk/NavLibrary.hpp>
#include <gnsstk/Position.hpp>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <gnsstk/OrbitData.hpp>
#include <gnss_info/cache.h>
#include <gnss_info/cache_index.h>
#include <gnss_info/nav_library_orbital_data_provider.h>
#include <gnsstk_ros/constellations.h>
#include <gnsstk_ros/position.h>
#include <gnsstk_ros/time.h>
#include <ros/ros.h>


namespace gnss_info
{

struct NavLibraryOrbitalDataProviderPrivate
{
    std::list<std::shared_ptr<NavLibraryDataSource>> sources;
    std::pair<ros::Time, ros::Time> timeRange;
    std::unordered_set<std::string> constellations;
    gnsstk::NavDataFactoryPtr factory;
    gnsstk::NavLibrary navLibrary;

    // Error of almanac positions is in the order of a few kilometers.
    std::unordered_map<gnsstk::NavMessageType, double> posCov = {
        {gnsstk::NavMessageType::Almanac, std::pow(2000.0, 2)},
        {gnsstk::NavMessageType::Ephemeris, std::pow(2.0, 2)},
    };
    std::unordered_map<gnsstk::NavMessageType, double> velCov = {
        {gnsstk::NavMessageType::Almanac, std::pow(1.0, 2)},
        {gnsstk::NavMessageType::Ephemeris, std::pow(0.1, 2)},
    };
};

NavLibraryOrbitalDataProvider::NavLibraryOrbitalDataProvider() : data(new NavLibraryOrbitalDataProviderPrivate)
{
    this->data->factory = std::make_shared<gnsstk::MultiFormatNavDataFactory>();
    this->data->navLibrary.addFactory(this->data->factory);
}

NavLibraryOrbitalDataProvider::~NavLibraryOrbitalDataProvider() = default;

void NavLibraryOrbitalDataProvider::addDataSource(const std::shared_ptr<NavLibraryDataSource>& source)
{
    this->data->sources.push_back(source);

    for (const auto& s : this->data->sources)
    {
        const auto& range = s->getTimeRange();

        if (this->data->timeRange.first == ros::Time::ZERO)
            this->data->timeRange.first = range.first;
        else
            this->data->timeRange.first = std::min(this->data->timeRange.first, range.first);

        if (this->data->timeRange.second == ros::Time::ZERO)
            this->data->timeRange.second = range.second;
        else
            this->data->timeRange.second = std::max(this->data->timeRange.second, range.second);

        this->data->constellations.insert(s->getConstellations().begin(), s->getConstellations().end());
    }
}

std::pair<ros::Time, ros::Time> NavLibraryOrbitalDataProvider::getTimeRange() const
{
    return this->data->timeRange;
}

std::unordered_set<std::string> NavLibraryOrbitalDataProvider::getConstellations() const
{
    return this->data->constellations;
}

bool NavLibraryOrbitalDataProvider::load(const ros::Time& time, const cras::optional<bool>& precise)
{
    bool success {false};
    for (const auto& source : this->data->sources)
    {
        if (precise.has_value() && *precise && !source->isPrecise())
            continue;
        if (precise.has_value() && !*precise && !source->isApproximate())
            continue;
        success |= source->load(
            time, [this](const std::string& file) {return this->data->factory->addDataSource(file);});
    }
    return success;
}

bool NavLibraryOrbitalDataProvider::load(const ros::Time& startTime, const ros::Time& endTime,
    const cras::optional<bool>& precise)
{
    bool success {false};
    for (const auto& source : this->data->sources)
    {
        if (precise.has_value() && *precise && !source->isPrecise())
            continue;
        if (precise.has_value() && !*precise && !source->isApproximate())
            continue;
        success |= source->load(startTime, endTime,
            [this](const std::string& file) {return this->data->factory->addDataSource(file);});
    }
    return success;
}

cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>, std::string>
NavLibraryOrbitalDataProvider::getECEFPositions(
    const ros::Time& time, const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satellites)
{
    const auto when = gnsstk_ros::convert(time);

    auto when1 = when, when2 = when;
    when1.addSeconds(-ros::Duration::DAY.sec * 3.0);
    when2.addSeconds(ros::Duration::DAY.sec * 3.0);
    const auto availableSats = this->data->navLibrary.getAvailableSats(when1, when2);
    if (availableSats.empty())
        return cras::make_unexpected("No matching satellite data found.");

    std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition> positions;
    if (satellites.empty())
        return positions;

    std::list<std::string> errors;
    for (const auto& [satcatID, info] : satellites)
    {
        const auto maybeSatID = gnsstk_ros::satelliteInfoToSatID(info);
        if (!maybeSatID.has_value())
        {
            errors.push_back(cras::format("Failed to determine PRN of satellite %u (%s) at time %s.",
                satcatID, info.name.c_str(), cras::to_string(time).c_str()));
            continue;
        }

        const gnsstk::NavSatelliteID navId{*maybeSatID};
        if (availableSats.find(navId) == availableSats.cend())
            continue;

        std::array<gnsstk::NavMessageID, 2> searchTerms {{
            {navId, gnsstk::NavMessageType::Ephemeris},
            {navId, gnsstk::NavMessageType::Almanac}
        }};
        gnsstk::NavDataPtr navData;
        gnsstk::NavMessageType foundMessageType;
        bool success {false};
        for (const auto& searchTerm : searchTerms)
        {
            if (this->data->navLibrary.find(searchTerm, when, navData,
                gnsstk::SVHealth::Any, gnsstk::NavValidityType::ValidOnly, gnsstk::NavSearchOrder::Nearest))
            {
                success = true;
                foundMessageType = searchTerm.messageType;
                break;
            }
        }

        if (!success || navData == nullptr)
        {
            errors.push_back(cras::format("No position data for satellite %u (%s) at time %s.",
                satcatID, info.name.c_str(), cras::to_string(time).c_str()));
            continue;
        }

        const auto orbitData = dynamic_cast<gnsstk::OrbitData*>(navData.get());
        gnsstk::Xvt xvt;
        success = orbitData->getXvt(when, xvt);

        if (!success)
        {
            errors.push_back(cras::format("Failed to compute ECEF position of satellite %u (%s) at time %s.",
                satcatID, info.name.c_str(), cras::to_string(time).c_str()));
            continue;
        }
        positions[satcatID] = gnsstk_ros::convert(
            xvt, satcatID, this->data->posCov[foundMessageType], this->data->velCov[foundMessageType]);
    }

    if (!errors.empty() && positions.empty())
        return cras::make_unexpected(cras::join(errors, " "));
    if (positions.empty())
        return cras::make_unexpected("No matching satellite data found.");
    if (!errors.empty())
        ROS_WARN("%s", cras::join(errors, " ").c_str());

    return positions;
}

cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition>, std::string>
NavLibraryOrbitalDataProvider::getSkyView(
const geographic_msgs::GeoPoint& position,
    const std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>& positions,
    const double elevationMaskDeg)
{
    gnsstk::Position recPos = gnsstk_ros::convert(position);
    // all following computations use Cartesian internally
    recPos.transformTo(gnsstk::Position::CoordinateSystem::Cartesian);

    std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition> skyView;
    for (const auto& [satcatID, ecefPose] : positions)
    {
        const gnsstk::Position satPos = gnsstk_ros::convert(ecefPose.position);
        const auto elDeg = recPos.elevation(satPos);
        if (elDeg < elevationMaskDeg)
            continue;
        const auto azDeg = recPos.azimuth(satPos);
        const auto distance = gnsstk::range(recPos, satPos);
        auto& skyPosition = skyView[satcatID];
        skyPosition.satcat_id = satcatID;
        skyPosition.azimuth_deg = azDeg;
        skyPosition.elevation_deg = elDeg;
        skyPosition.distance = distance;
    }

    return skyView;
}

bool NavLibraryOrbitalDataProvider::isPrecise() const
{
    return std::any_of(this->data->sources.cbegin(), this->data->sources.cend(),
        [](const std::shared_ptr<NavLibraryDataSource>& s) {return s->isPrecise();});
}

bool NavLibraryOrbitalDataProvider::isApproximate() const
{
    return std::any_of(this->data->sources.cbegin(), this->data->sources.cend(),
        [](const std::shared_ptr<NavLibraryDataSource>& s) {return s->isApproximate();});
}

std::string NavLibraryOrbitalDataProvider::getName() const
{
    std::list<std::string> names;
    std::transform(this->data->sources.cbegin(), this->data->sources.cend(),
        std::back_inserter(names), [](auto s) {return s->getName();});
    return "GNSSTk NavLibrary (" + cras::join(names, ", ") + ")";
}

}
