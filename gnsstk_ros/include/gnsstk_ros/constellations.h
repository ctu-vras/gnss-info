// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <string>
#include <utility>

#include <gnsstk/SatID.hpp>
#include <gnsstk/SatelliteSystem.hpp>

#include <cras_cpp_common/optional.hpp>
#include <gnss_info_msgs/SatelliteInfo.h>

namespace gnsstk_ros
{

/**
 * \brief Get constellation name from the given PRN string like E03.
 * \param[in] prn String PRN.
 * \return Constellation name (one of `gnss_info_msgs::Enums::CONSTELLATION_*` values). If invalid, nullopt.
 */
cras::optional<std::string> getRosConstellationFromPRN(const std::string& prn);

/**
 * \brief Get constellation name from the given SVN string like E203.
 * \param[in] svn The SVN of the satellite.
 * \return Constellation name (one of `gnss_info_msgs::Enums::CONSTELLATION_*` values). If invalid, nullopt.
 */
cras::optional<std::string> getRosConstellationFromSVN(const std::string& svn);

/**
 * \brief Convert the given PRN string like E03 to integer like 3 and constellation name.
 * \param[in] prn The PRN string.
 * \return A pair containing the numeric PRN and constellation name.
 */
cras::optional<std::pair<int32_t, std::string>> prnStringToInt(const std::string& prn);

/**
 * \brief Convert the given numeric PRN and constellation to string PRN like E03.
 * \param[in] prn The numeric PRN.
 * \param[in] constellation Constellation name (one of `gnss_info_msgs::Enums::CONSTELLATION_*` values).
 * \return The PRN string.
 */
cras::optional<std::string> prnIntToString(int32_t prn, const std::string& constellation);

/**
 * \brief Convert the gnss_info_msgs constellation string to gnsstk constellation enum.
 * \param[in] constellation The constellation string. One of `gnss_info_msgs::Enums::CONSTELLATION_*`.
 * \return The equivalent gnsstk constellation. Return `Unknown` if there is no mapping. Return `nullopt` if
 *         `constellation` is empty.
 */
cras::optional<gnsstk::SatelliteSystem> rosConstellationToGnsstkSatelliteSystem(const std::string& constellation);

/**
 * \brief Convert the gnsstk constellation enum to gnss_info_msgs constellation string.
 * \param constellation The gnsstk constellation enum value.
 * \return The equivalent `gnss_info_msgs::Enums::CONSTELLATION_*`. Return `nullopt` if there is no mapping.
 */
cras::optional<std::string> gnsstkSatelliteSystemRosConstellation(const gnsstk::SatelliteSystem& constellation);

/**
 * \brief Convert the given satellite info to non-wildcard gnsstk SatID.
 * \param info Information about the satellite.
 * \return The SatID object of the given satellite. `nullopt` if identifying the satellite is not possible.
 */
cras::optional<gnsstk::SatID> satelliteInfoToSatID(const gnss_info_msgs::SatelliteInfo& info);

}
