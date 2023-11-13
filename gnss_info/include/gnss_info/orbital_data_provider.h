// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
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

    /**
     * \brief Get human-readable name of the provider.
     * \return The name.
     */
    virtual std::string getName() const = 0;

    /**
     * \brief Return whether this provider works with precise orbit data.
     * \return Whether this provider works with precise orbit data.
     */
    virtual bool isPrecise() const = 0;

    /**
     * \brief Return whether this provider works with approximate orbit data.
     * \return Whether this provider works with approximate orbit data.
     */
    virtual bool isApproximate() const = 0;

    /**
     * \brief Get the time range in which this provider can provide information.
     * \return The time range (first, latest).
     */
    virtual std::pair<ros::Time, ros::Time> getTimeRange() const = 0;

    /**
     * \brief Get the constellations handled by this provider.
     * \return The constellations.
     */
    virtual std::unordered_set<std::string> getConstellations() const = 0;

    /**
     * \brief Load data for the given time instant.
     * \param[in] time The time to load at.
     * \param[in] precise If set, selects whether precise or approximate data should be loaded.
     * \return Whether loading succeeded.
     */
    virtual bool load(const ros::Time& time, const cras::optional<bool>& precise) = 0;

    /**
     * \brief Load data for the given time instant.
     * \param[in] startTime Earliest time to load.
     * \param[in] endTime Latest time to load.
     * \param[in] precise If set, selects whether precise or approximate data should be loaded.
     * \return Whether loading succeeded.
     */
    virtual bool load(const ros::Time& startTime, const ros::Time& endTime, const cras::optional<bool>& precise) = 0;

    /**
     * \brief Compute ECEF positions of the satellites at the given time.
     * \param[in] time The time to get positions for.
     * \param[in] satellites The satellites to get positions for.
     * \return ECEF positions of the satellites, or an error string.
     */
    virtual cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>, std::string>
    getECEFPositions(const ros::Time& time,
        const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satellites) = 0;

    /**
     * \brief Compute sky view (azimuths, elevations and distances) of satellites from the given receiver position.
     * \param[in] position Geographic position of the receiver.
     * \param[in] positions ECEF positions of the satellites.
     * \param[in] elevationMaskDeg Minimum elevation of satellites to return (in degrees).
     * \return Sky view corresponding to the given configuration, or an error string.
     */
    virtual cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition>, std::string> getSkyView(
        const geographic_msgs::GeoPoint& position,
        const std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>& positions,
        double elevationMaskDeg) = 0;
};

}
