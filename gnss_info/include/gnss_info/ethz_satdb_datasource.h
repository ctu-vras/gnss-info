// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gnss_info/nav_library_data_source.h>
#include <gnss_info_msgs/SatelliteInfo.h>

#include <ros/time.h>

namespace gnss_info
{

struct EthzSatdbDataSourcePrivate;

/**
 * \brief GNSSTk NavLibrary data source downloading TLE data from ETHZ Satellite Database.
 */
class EthzSatdbDataSource : public NavLibraryDataSource
{
public:
    explicit EthzSatdbDataSource(const std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo>& satelliteInfo);
    ~EthzSatdbDataSource() override;

    std::string getName() const override;
    bool isPrecise() const override;
    bool isApproximate() const override;

    std::pair<ros::Time, ros::Time> getTimeRange() const override;
    std::unordered_set<std::string> getConstellations() const override;

    bool load(const ros::Time& time, const DataSourceLoadCb& cb) override;
    bool load(const ros::Time& startTime, const ros::Time& endTime, const DataSourceLoadCb& cb) override;

private:
    std::unique_ptr<EthzSatdbDataSourcePrivate> data;  //!< Private implementation data (PIMPL).
};

}
