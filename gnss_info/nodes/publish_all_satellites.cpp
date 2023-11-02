// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * An example node using IGSSatelliteMetadata to construct a list of all GNSS satellites that existed in a given
 * time instant.
 *
 * Published topics:
 * - `satellites` (`gnss_info_msgs/SatellitesList`, latched): The list of satellites.
 *
 * Parameters:
 * - `~time` (float, defaults to ros::Time::now() ): The reference time for which satellites should be looked up.
 * - `~only_active` (bool, default true): Return only satellites active at the reference time.
 * - `~only_constellations` (list of string): If nonempty, limits the published satellites to only those belonging to
 *                                            the listed constellations.
 * - `~only_signals` (list of string): If nonempty, limits the published satellites to only those transmitting at least
 *                                     one of the listed signals.
 */

#include <string>
#include <unordered_set>

#include <cras_cpp_common/node_utils.hpp>
#include <gnss_info/igs_satellite_metadata.h>
#include <gnss_info_msgs/Enums.h>
#include <gnss_info_msgs/SatellitesList.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_all_satellites");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    const auto params = cras::nodeParams(pnh);

    auto pub = nh.advertise<gnss_info_msgs::SatellitesList>("satellites", 1, true);

    const auto time = params->getParam("time", ros::Time::now());
    const auto onlyActive = params->getParam("only_active", true);
    const auto onlyConstellations = params->getParam("only_constellations", std::unordered_set<std::string>());
    const auto onlySignals = params->getParam("only_signals", std::unordered_set<std::string>());

    gnss_info::IGSSatelliteMetadata metadata;
    if (!metadata.load())
    {
        ROS_ERROR("Failed to load satellite metadata!");
    }
    else
    {
        const auto sats = metadata.getSatellites(time, onlyActive, onlyConstellations, onlySignals);
        gnss_info_msgs::SatellitesList msg;
        for (const auto& pair : sats)
            msg.satellites.push_back(pair.second);
        ROS_INFO("Publishing %zu satellites.", sats.size());
        pub.publish(msg);
        ros::spin();
    }
}
