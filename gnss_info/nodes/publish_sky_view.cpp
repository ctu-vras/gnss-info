// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * An example node using OrbitalDataManager to construct a sky view of satellites.
 *
 * Published topics:
 * - `satellites_positions` (`gnss_info_msgs/SatellitesPositions`): The ECEF positions of all satellites.
 * - `sky_view` (`gnss_info_msgs/SkyView`): The sky view.
 *
 * Subscribed topics:
 * - `position` (`geographic_msgs/GeoPoint`): The reference position.
 * - `satellites` (`gnss_info_msgs/SatellitesList`): The list of all active satellites.
 *
 * Parameters:
 * - `~elevation_mask_deg` (double, default 5.0): Minimum elevation of satellites to consider
 *                                                (in degrees above horizon).
 */

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <cras_cpp_common/node_utils.hpp>
#include <gnss_info/ethz_satdb_datasource.h>
#include <gnss_info/nav_library_orbital_data_provider.h>
#include <gnss_info/orbital_data_manager.h>
#include <gnss_info_msgs/Enums.h>
#include <gnss_info_msgs/SatellitesList.h>
#include <gnss_info_msgs/SatellitesPositions.h>
#include <gnss_info_msgs/SkyView.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_sky_view");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    const auto params = cras::nodeParams(pnh);

    const auto elevationMaskDeg = params->getParam("elevation_mask_deg", 5.0);

    gnss_info::OrbitalDataManager manager;
    gnss_info_msgs::SatellitesListConstPtr satellites;
    geographic_msgs::GeoPointConstPtr lastPosition;

    auto positionsPub = nh.advertise<gnss_info_msgs::SatellitesPositions>("satellites_positions", 1, true);
    auto skyViewPub = nh.advertise<gnss_info_msgs::SkyView>("sky_view", 1, true);

    auto satellitesCallback = [&manager, &satellites](const gnss_info_msgs::SatellitesListConstPtr& list)
    {
        if (satellites != nullptr && !satellites->satellites.empty())
            return;
        satellites = list;

        std::unordered_map<uint32_t, gnss_info_msgs::SatelliteInfo> infos;
        for (const auto& sat : list->satellites)
            infos[sat.satcat_id] = sat;

        const auto provider = std::make_shared<gnss_info::NavLibraryOrbitalDataProvider>();
        manager.addProvider(provider);
        provider->addDataSource(std::make_shared<gnss_info::EthzSatdbDataSource>(infos));
        ROS_INFO("Added satellite orbit data provider %s.", provider->getName().c_str());

        ROS_INFO("Loading data.");
        manager.load(ros::Time::now(), ros::Time::now() + ros::Duration::DAY);
        ROS_INFO("Loading data finished.");
    };
    auto satSub = nh.subscribe<gnss_info_msgs::SatellitesList>("satellites", 1, satellitesCallback);

    ros::Timer timer;
    const auto refreshRate = cras::safeRate(params->getParam("refresh_rate", 0.1, "Hz"));
    const auto refreshPeriod = refreshRate.expectedCycleTime();

    auto positionCallback = [&](const geographic_msgs::GeoPointConstPtr& position)
    {
        lastPosition = position;
        if (satellites == nullptr || satellites->satellites.empty())
        {
            ROS_WARN_DELAYED_THROTTLE(1.0, "Waiting for satellites metadata on topic %s.",
                                      nh.resolveName(satSub.getTopic()).c_str());
            return;
        }

        const auto maybePositions = manager.getPositions(ros::Time::now(), *satellites);
        if (!maybePositions.has_value())
        {
            ROS_ERROR_THROTTLE(1.0, "%s", maybePositions.error().c_str());
            return;
        }
        positionsPub.publish(*maybePositions);

        const auto maybeSkyView = manager.getSkyView(*position, *maybePositions, elevationMaskDeg);
        if (!maybeSkyView.has_value())
        {
            ROS_ERROR_THROTTLE(1.0, "%s", maybeSkyView.error().c_str());
            return;
        }
        skyViewPub.publish(*maybeSkyView);

        ROS_DEBUG("Computed sky view for time %s with %zu/%zu satellites in view.",
                  cras::to_string(maybeSkyView->header.stamp).c_str(),
                  maybeSkyView->satellites.size(), maybePositions->satellites.size());

        timer.setPeriod(refreshPeriod);
    };
    auto posSub = nh.subscribe<geographic_msgs::GeoPoint>("position", 1, positionCallback);

    auto timerCb = [&](const ros::TimerEvent&)
    {
        positionCallback(lastPosition);
    };
    timer = nh.createTimer(refreshPeriod, timerCb);

    ROS_INFO("Started sky view generator with elevation mask %0.0f degrees.", elevationMaskDeg);
    ros::spin();
}
