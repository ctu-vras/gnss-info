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

/**
 * \brief Manager of multiple providers of satellite orbital data.
 */
class OrbitalDataManager
{
public:
    OrbitalDataManager();
    virtual ~OrbitalDataManager();

    /**
     * \brief Add the given provider to this manager and make its data available.
     * \param[in] provider The provider to add.
     */
    void addProvider(const std::shared_ptr<OrbitalDataProvider>& provider);

    /**
     * \brief Load (and possibly download) data for the given time instant.
     * \param[in] time The time instant to load for.
     * \return Whether the loading succeeded.
     */
    bool load(const ros::Time& time);

    /**
     * \brief Load (and possibly download) data for the given time instant.
     * \param[in] time The time instant to load for.
     * \param[in] precise If set, selects whether precise or approximate data should be loaded.
     * \return Whether the loading succeeded.
     */
    virtual bool load(const ros::Time& time, const cras::optional<bool>& precise);

    /**
     * \brief Load (and possibly download) data for the given time interval.
     * \param[in] startTime Earliest time to load.
     * \param[in] endTime Latest time to load.
     * \return Whether the loading succeeded.
     */
    bool load(const ros::Time& startTime, const ros::Time& endTime);

    /**
     * \brief Load (and possibly download) data for the given time interval.
     * \param[in] startTime Earliest time to load.
     * \param[in] endTime Latest time to load.
     * \param[in] precise If set, selects whether precise or approximate data should be loaded.
     * \return Whether the loading succeeded.
     */
    virtual bool load(const ros::Time& startTime, const ros::Time& endTime, const cras::optional<bool>& precise);

    /**
     * \brief Compute ECEF positions of the satellites at the given time.
     * \param[in] time The time to get positions for.
     * \param[in] satellites The satellites to get positions for.
     * \return ECEF positions of the satellites, or an error string.
     */
    cras::expected<gnss_info_msgs::SatellitesPositions, std::string> getPositions(
        const ros::Time& time, const gnss_info_msgs::SatellitesList& satellites);

    /**
     * \brief Compute ECEF positions of the satellites at the given time.
     * \param[in] time The time to get positions for.
     * \param[in] satellites The satellites to get positions for.
     * \param[in] precise If set, selects whether precise or approximate positions should be computed.
     * \return ECEF positions of the satellites, or an error string.
     */
    virtual cras::expected<gnss_info_msgs::SatellitesPositions, std::string> getPositions(
        const ros::Time& time, const gnss_info_msgs::SatellitesList& satellites, const cras::optional<bool>& precise);

    /**
     * \brief Compute sky view (azimuths, elevations and distances) of satellites from the given receiver position.
     * \param[in] position Geographic position of the receiver.
     * \param[in] positions ECEF positions of the satellites.
     * \param[in] elevationMaskDeg Minimum elevation of satellites to return (in degrees).
     * \return Sky view corresponding to the given configuration, or an error string.
     */
    cras::expected<gnss_info_msgs::SkyView, std::string> getSkyView(
        const geographic_msgs::GeoPoint& position, const gnss_info_msgs::SatellitesPositions& positions,
        double elevationMaskDeg);

    /**
     * \brief Compute sky view (azimuths, elevations and distances) of satellites from the given receiver position.
     * \param[in] position Geographic position of the receiver.
     * \param[in] positions ECEF positions of the satellites.
     * \param[in] elevationMaskDeg Minimum elevation of satellites to return (in degrees).
     * \param[in] precise If set, selects whether precise or approximate positions should be computed.
     * \return Sky view corresponding to the given configuration, or an error string.
     */
    virtual cras::expected<gnss_info_msgs::SkyView, std::string> getSkyView(
        const geographic_msgs::GeoPoint& position, const gnss_info_msgs::SatellitesPositions& positions,
        double elevationMaskDeg, const cras::optional<bool>& precise);

    /**
     * \brief Compute Dilution of Precision for the given sky view.
     * \param[in] skyView The sky view to compute DOP for.
     * \return The dilution of precision for the given sky view.
     */
    virtual gnss_info_msgs::DOP computeDOP(
        const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition>& skyView) const;

protected:
    std::unique_ptr<OrbitalDataManagerPrivate> data;  //!< Private implementation details (PIMPL).
};

}
