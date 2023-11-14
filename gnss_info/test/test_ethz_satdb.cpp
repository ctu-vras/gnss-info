// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Test for OrbitalDataManger with EthzSatdbDatasource.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cstdlib>
#include <string>

#include <gnss_info/ethz_satdb_datasource.h>
#include <gnss_info/igs_satellite_metadata.h>
#include <gnss_info/nav_library_orbital_data_provider.h>
#include <gnss_info/orbital_data_manager.h>
#include <gnss_info_msgs/Enums.h>
#include <gnss_info_msgs/SatellitesList.h>
#include <ros/time.h>

using namespace gnss_info;
using namespace gnss_info_msgs;

TEST(OrbitalDataProvider, EthzSatdb)  // NOLINT
{
    IGSSatelliteMetadata metadata;
    ASSERT_TRUE(metadata.load());

    ros::Time time {1699955783, 0};
    const auto& infos = metadata.getSatellites(time);
    SatellitesList satellites;
    for (const auto& [satcatId, info] : infos)
        satellites.satellites.push_back(info);

    OrbitalDataManager manager;
    const auto provider = std::make_shared<NavLibraryOrbitalDataProvider>();
    manager.addProvider(provider);
    provider->addDataSource(std::make_shared<EthzSatdbDataSource>(infos));
    manager.load(time);

    const auto maybePositions = manager.getPositions(time, satellites);
    ASSERT_TRUE(maybePositions.has_value());
    const auto& positions = *maybePositions;

    EXPECT_EQ(positions.satellites.size(), 118u);
    EXPECT_EQ(positions.header.stamp, time);
    EXPECT_EQ(positions.header.frame_id, "ecef");
    ASSERT_FALSE(positions.satellites.empty());

    EXPECT_EQ(positions.satellites.at(0).satcat_id, 43603);
    EXPECT_NEAR(positions.satellites.at(0).position.x, 24525463, 1.0);
    EXPECT_NEAR(positions.satellites.at(0).position.y, -4510264, 1.0);
    EXPECT_NEAR(positions.satellites.at(0).position.z, -12515116, 1.0);
    EXPECT_NEAR(positions.satellites.at(0).velocity.x, 1378, 1.0);
    EXPECT_NEAR(positions.satellites.at(0).velocity.y, 410, 1.0);
    EXPECT_NEAR(positions.satellites.at(0).velocity.z, 2557, 1.0);
    EXPECT_FLOAT_EQ(positions.satellites.at(0).position_covariance[0], 4000000.0);
    EXPECT_FLOAT_EQ(positions.satellites.at(0).position_covariance[4], 4000000.0);
    EXPECT_FLOAT_EQ(positions.satellites.at(0).position_covariance[8], 4000000.0);
    EXPECT_FLOAT_EQ(positions.satellites.at(0).velocity_covariance[0], 1.0);
    EXPECT_FLOAT_EQ(positions.satellites.at(0).velocity_covariance[4], 1.0);
    EXPECT_FLOAT_EQ(positions.satellites.at(0).velocity_covariance[8], 1.0);

    geographic_msgs::GeoPoint position;
    position.latitude = 50.0;
    position.longitude = 14.4;
    position.altitude = 200.0;
    const auto maybeSkyView = manager.getSkyView(position, positions, 5.0);
    ASSERT_TRUE(maybeSkyView.has_value());
    const auto& skyView = *maybeSkyView;

    EXPECT_EQ(skyView.header.stamp, time);
    EXPECT_EQ(skyView.header.frame_id, "WGS84");
    EXPECT_EQ(skyView.reference_position, position);
    EXPECT_FLOAT_EQ(skyView.dop.gdop, 0.77478588);
    EXPECT_FLOAT_EQ(skyView.dop.pdop, 0.7113663);
    EXPECT_FLOAT_EQ(skyView.dop.hdop, 0.41898307);
    EXPECT_FLOAT_EQ(skyView.dop.vdop, 0.5748871);
    EXPECT_FLOAT_EQ(skyView.dop.tdop, 0.30700356);
    EXPECT_FLOAT_EQ(skyView.elevation_mask_deg, 5.0);
    ASSERT_FALSE(skyView.satellites.empty());
    EXPECT_EQ(skyView.satellites.size(), 41u);

    EXPECT_EQ(skyView.satellites.at(0).satcat_id, 49810);
    EXPECT_FLOAT_EQ(skyView.satellites.at(0).azimuth_deg, 306.88858);
    EXPECT_FLOAT_EQ(skyView.satellites.at(0).elevation_deg, 28.904871);
    EXPECT_FLOAT_EQ(skyView.satellites.at(0).distance, 25989960);

    for (const auto& sat : skyView.satellites)
    {
        EXPECT_GE(sat.elevation_deg, 5.0);
    }
}

int main(int argc, char** argv)
{
    setenv("GNSS_INFO_CACHE_DIR", TEST_CACHE_DIR, true);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
