// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <cras_cpp_common/optional.hpp>
#include <gnss_info_msgs/SatelliteInfo.h>

#include <ros/duration.h>
#include <ros/time.h>

namespace gnss_info
{

/**
 * \brief Get constellation name from the given PRN string like E03.
 * \param[in] prn String PRN.
 * \return Constellation name (one of `gnss_info_msgs::Enums::CONSTELLATION_*` values). If invalid, empty string.
 */
std::string getConstellationFromPRN(const std::string& prn);

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

class IGSSatelliteMetadataPrivate;

/**
 * \brief Wrapper around IGS satellite catalog which contains information about all public GNSS satellites.
 *
 * The following environment variables are considered. If not set, a sensible default is used.
 * - `GNSS_INFO_CACHE_DIR`: The directory into which cache will be downloaded.
 *  - If not specified, the default tries `XDG_CACHE_HOME` and otherwise `~/.cache`.
 * - `GNSS_INFO_IGS_METADATA_URL`: The URL from which the satellite catalog should be downloaded.
 * - `GNSS_INFO_SIGNALS_PATH`: The directories containing the YAML files defining which signals are transmitted by which
 *                             satellite blocks. Either a single directory path, or a colon-delimited path list.
 */
class IGSSatelliteMetadata
{
public:
    IGSSatelliteMetadata();
    virtual ~IGSSatelliteMetadata();

    /**
     * \brief Set the URL from which IGS metadata are downloaded.
     *
     * This has no effect if load() has already been called.
     *
     * \param[in] url The new URL.
     */
    void setMetadataURL(const std::string& url);

    /**
     * \brief Set the cache file into which the IGS metadata are downloaded.
     *
     * This has no effect if load() has already been called.
     *
     * \param[in] file The new file.
     */
    void setCacheFile(const std::string& file);

    /**
     * \brief Set validity period of the cached IGS metadata.
     *
     * \param[in] validity How long the cache will be valid.
     */
    void setCacheValidity(const ros::WallDuration& validity);

    /**
     * \brief Load (and possibly also download) the data.
     *
     * \return Whether loading succeeded. If false, the metadata is not loaded and the class should not be used.
     */
    bool load();

    /**
     * \brief Get all known satellites operating at the given time. If load() was not called or returned false, empty
     *        list will be returned.
     * \param[in] time The reference time (epoch).
     * \param[in] onlyActive If true, only active satellites will be returned.
     * \return Mapping Satcat ID => known valid satellite.
     */
    std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo> getSatellites(
        const ros::Time& time, bool onlyActive = true, const std::unordered_set<std::string>& onlyConstellations = {},
        const std::unordered_set<std::string>& onlySignals = {});

    /**
     * \brief Get metadata of a satellite at the given time.
     * \param[in] satcatID The satcat ID of the satellite.
     * \param[in] time The reference time (epoch).
     * \return The satellite metadata. If load() was not called or returned false, nullopt will be returned.
     *         If a satellite with the given ID does not exist, nullopt is returned. If the satellite exists but is not
     *         operational at the reference time, it will be returned with `active` set to false.
     */
    cras::optional<gnss_info_msgs::SatelliteInfo> getSatellite(const uint32_t& satcatID, const ros::Time& time);

    /**
     * \brief Get metadata of a satellite at the given time.
     * \param[in] prn The PRN of the satellite at the reference time. This should be the IGS catalog-style PRN that
     *                starts with constellation letter, e.g. E001 for Galileo PRN 1.
     * \param[in] time The reference time (epoch).
     * \return The satellite metadata. If load() was not called or returned false, nullopt will be returned.
     *         If a satellite with the given PRN does not exist, nullopt is returned. If the satellite exists but is not
     *         operational at the reference time, it will be returned with `active` set to false.
     */
    cras::optional<gnss_info_msgs::SatelliteInfo> getSatelliteByPRN(const std::string& prn, const ros::Time& time);

    /**
     * \brief Get metadata of a satellite at the given time.
     * \param[in] prn The PRN of the satellite at the reference time. This should be the numeric PRN only, without any
     *                artificial offsets like those gpsd or NMEA add. However, the algorithm will try its best if it
     *                encounters some non-standard PRN.
     * \param[in] constellation The constellation the satellite belongs to.
     * \param[in] time The reference time (epoch).
     * \return The satellite metadata. If load() was not called or returned false, nullopt will be returned.
     *         If a satellite with the given PRN does not exist, nullopt is returned. If the satellite exists but is not
     *         operational at the reference time, it will be returned with `active` set to false.
     */
    cras::optional<gnss_info_msgs::SatelliteInfo> getSatelliteByPRN(
        int32_t prn, const std::string& constellation, const ros::Time& time);

protected:
    std::unique_ptr<IGSSatelliteMetadataPrivate> data;  //!< Private implementation details (PIMPL).
};

}
