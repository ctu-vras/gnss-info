// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <cras_cpp_common/expected.hpp>
#include <gnss_info/nav_library_data_source.h>
#include <gnss_info/orbital_data_provider.h>
#include <ros/ros.h>


namespace gnss_info
{

struct NavLibraryOrbitalDataProviderPrivate;

/**
 * \brief GNSS satellite orbits provider based on gnsstk::NavLibrary with multi-format data source.
 */
class NavLibraryOrbitalDataProvider : public OrbitalDataProvider
{
public:
    NavLibraryOrbitalDataProvider();
    ~NavLibraryOrbitalDataProvider() override;

    /**
     * \brief Add data source.
     * \param[in] source The data source to add.
     */
    virtual void addDataSource(const std::shared_ptr<NavLibraryDataSource>& source);

    std::string getName() const override;
    bool isPrecise() const override;
    bool isApproximate() const override;

    std::pair<ros::Time, ros::Time> getTimeRange() const override;
    std::unordered_set<std::string> getConstellations() const override;

    bool load(const ros::Time& time, const cras::optional<bool>& precise) override;
    bool load(const ros::Time& startTime, const ros::Time& endTime, const cras::optional<bool>& precise) override;

    cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>, std::string> getECEFPositions(
        const ros::Time& time, const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satellites) override;

    cras::expected<std::unordered_map<uint32_t, gnss_info_msgs::SatelliteSkyPosition>, std::string> getSkyView(
        const geographic_msgs::GeoPoint& position,
        const std::unordered_map<uint32_t, gnss_info_msgs::SatellitePosition>& positions,
        double elevationMaskDeg) override;

private:
    std::unique_ptr<NavLibraryOrbitalDataProviderPrivate> data;  //!< Private implementation data (PIMPL).
};

}
