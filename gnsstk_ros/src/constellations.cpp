// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <string>
#include <utility>

#include <gnsstk/SatID.hpp>

#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <gnsstk_ros/constellations.h>
#include <gnss_info_msgs/Enums.h>
#include <gnss_info_msgs/SatelliteInfo.h>

namespace gnsstk_ros
{

cras::optional<std::string> getRosConstellationFromPRN(const std::string& prn)
{
    if (prn.empty())
        return cras::nullopt;

    switch (prn[0])
    {
        case 'G':
            return gnss_info_msgs::Enums::CONSTELLATION_GPS;
        case 'R':
            return gnss_info_msgs::Enums::CONSTELLATION_GLONASS;
        case 'E':
            return gnss_info_msgs::Enums::CONSTELLATION_GALILEO;
        case 'C':
            return gnss_info_msgs::Enums::CONSTELLATION_BEIDOU;
        case 'J':
            return gnss_info_msgs::Enums::CONSTELLATION_QZSS;
        case 'I':
            return gnss_info_msgs::Enums::CONSTELLATION_NAVIC;
        default:
            return cras::nullopt;
    }
}

cras::optional<std::string> getRosConstellationFromSVN(const std::string& svn)
{
    // The implementation is actually the same.
    return getRosConstellationFromPRN(svn);
}

cras::optional<std::pair<int32_t, std::string>> prnStringToInt(const std::string& prn)
{
    const auto constellation = getRosConstellationFromPRN(prn);
    if (!constellation.has_value())
        return cras::nullopt;

    auto prnString = prn.substr(1);

    // Get rid of leading zeros, they would trick parseInt32 into parsing as octal
    while (!prnString.empty() && prnString[0] == '0')
        prnString = prnString.substr(1);
    if (prnString.empty())
        return cras::nullopt;

    try
    {
        return std::make_pair(cras::parseInt32(prnString), *constellation);
    }
    catch (const std::invalid_argument&)  // integer parsing failed
    {
        return cras::nullopt;
    }
}

cras::optional<std::string> prnIntToString(const int32_t prn, const std::string& constellation)
{
    std::string prefix;

    if (constellation == gnss_info_msgs::Enums::CONSTELLATION_GPS)
        prefix = "G";
    else if (constellation == gnss_info_msgs::Enums::CONSTELLATION_GLONASS)
        prefix = "R";
    else if (constellation == gnss_info_msgs::Enums::CONSTELLATION_GALILEO)
        prefix = "E";
    else if (constellation == gnss_info_msgs::Enums::CONSTELLATION_BEIDOU)
        prefix = "C";
    else if (constellation == gnss_info_msgs::Enums::CONSTELLATION_QZSS)
        prefix = "J";
    else if (constellation == gnss_info_msgs::Enums::CONSTELLATION_NAVIC)
        prefix = "I";
    else
        return cras::nullopt;

    return prefix + std::to_string(prn);
}

cras::optional<gnsstk::SatelliteSystem> rosConstellationToGnsstkSatelliteSystem(const std::string& constellation)
{
    if (constellation.empty())
        return gnsstk::SatelliteSystem::Unknown;
    if (constellation == gnss_info_msgs::Enums::CONSTELLATION_GPS)
        return gnsstk::SatelliteSystem::GPS;
    if (constellation == gnss_info_msgs::Enums::CONSTELLATION_GALILEO)
        return gnsstk::SatelliteSystem::Galileo;
    if (constellation == gnss_info_msgs::Enums::CONSTELLATION_GLONASS)
        return gnsstk::SatelliteSystem::Glonass;
    if (constellation == gnss_info_msgs::Enums::CONSTELLATION_BEIDOU)
        return gnsstk::SatelliteSystem::BeiDou;
    if (constellation == gnss_info_msgs::Enums::CONSTELLATION_NAVIC)
        return gnsstk::SatelliteSystem::IRNSS;
    if (constellation == gnss_info_msgs::Enums::CONSTELLATION_QZSS)
        return gnsstk::SatelliteSystem::QZSS;
    return gnsstk::SatelliteSystem::Unknown;
}

cras::optional<std::string> gnsstkSatelliteSystemRosConstellation(const gnsstk::SatelliteSystem& constellation)
{
    switch (constellation)
    {
        case gnsstk::SatelliteSystem::GPS:
            return gnss_info_msgs::Enums::CONSTELLATION_GPS;
        case gnsstk::SatelliteSystem::Galileo:
            return gnss_info_msgs::Enums::CONSTELLATION_GALILEO;
        case gnsstk::SatelliteSystem::Glonass:
            return gnss_info_msgs::Enums::CONSTELLATION_GLONASS;
        case gnsstk::SatelliteSystem::BeiDou:
            return gnss_info_msgs::Enums::CONSTELLATION_BEIDOU;
        case gnsstk::SatelliteSystem::IRNSS:
            return gnss_info_msgs::Enums::CONSTELLATION_NAVIC;
        case gnsstk::SatelliteSystem::QZSS:
            return gnss_info_msgs::Enums::CONSTELLATION_QZSS;
        default:
            return cras::nullopt;
    }
}

cras::optional<gnsstk::SatID> satelliteInfoToSatID(const gnss_info_msgs::SatelliteInfo& info)
{
    const auto maybeConstellationAndPrn = prnStringToInt(info.prn);
    if (!maybeConstellationAndPrn.has_value())
        return cras::nullopt;

    const auto& [prn, constellationStr] = *maybeConstellationAndPrn;

    auto maybeSatelliteSystem = rosConstellationToGnsstkSatelliteSystem(constellationStr);
    if (!maybeSatelliteSystem.has_value())
        maybeSatelliteSystem = rosConstellationToGnsstkSatelliteSystem(info.constellation);
    if (!maybeSatelliteSystem.has_value())
        return cras::nullopt;

    const auto system = *maybeSatelliteSystem;
    return gnsstk::SatID(prn, system);
}

}
