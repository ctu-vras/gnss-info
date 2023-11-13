// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <functional>
#include <string>
#include <unordered_set>
#include <utility>

#include <ros/time.h>


namespace gnss_info
{

/**
 * \brief Generic interface for various data sources for gnsstk::NavLibrary.
 */
class NavLibraryDataSource
{
public:
    /**
     * \brief Callback to be called when a new source file is introduced.
     * \param[in] file Path to the source file.
     * \return Whether reading the source file succeeded.
     */
    typedef std::function<bool(const std::string& file)> DataSourceLoadCb;

    /**
     * \brief Construct the data source.
     *
     * Also make sure the data source is registered via gnsstk::MultiFormatNavDataFactory::addFactory() (but only once).
     */
    NavLibraryDataSource() = default;

    virtual ~NavLibraryDataSource() = default;

    /**
     * \brief Get human-readable name of the data source.
     * \return The name.
     */
    virtual std::string getName() const = 0;

    /**
     * \brief Return whether this datasource works with precise orbit data.
     * \return Whether this datasource works with precise orbit data.
     */
    virtual bool isPrecise() const = 0;

    /**
     * \brief Return whether this datasource works with approximate orbit data.
     * \return Whether this datasource works with approximate orbit data.
     */
    virtual bool isApproximate() const = 0;

    /**
     * \brief Get the time range in which this datasource can provide information.
     * \return The time range (first, latest).
     */
    virtual std::pair<ros::Time, ros::Time> getTimeRange() const = 0;

    /**
     * \brief Get the constellations handled by this data source.
     * \return The constellations.
     */
    virtual std::unordered_set<std::string> getConstellations() const = 0;

    /**
     * \brief Load data for the given time instant.
     * \param[in] time The time to load at.
     * \param[in] cb Callback to call for each found and downloaded data file.
     * \return Whether loading succeeded.
     */
    virtual bool load(const ros::Time& time, const DataSourceLoadCb& cb) = 0;

    /**
     * \brief Load data for the given time interval.
     * \param[in] startTime The time to start loading at.
     * \param[in] endTime The time to stop loading at.
     * \param[in] cb Callback to call for each found and downloaded data file.
     * \return Whether loading succeeded.
     */
    virtual bool load(const ros::Time& startTime, const ros::Time& endTime, const DataSourceLoadCb& cb) = 0;
};

}
