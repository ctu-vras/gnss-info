// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Test for IgsSatelliteMetadataProvider.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cstdlib>
#include <string>

#include <gnss_info/igs_satellite_metadata.h>
#include <gnss_info_msgs/Enums.h>
#include <ros/time.h>

using namespace gnss_info;
using namespace gnss_info_msgs;

TEST(IgsSatelliteMetadata, LoadAll)  // NOLINT
{
    IGSSatelliteMetadata metadata;
    ASSERT_TRUE(metadata.load());

    ros::Time time {1699955783, 0};
    const auto sats = metadata.getSatellites(time);
    ASSERT_EQ(sats.size(), 151u);

    // Beidou test satellites are up there but do not transmit any known signal
    std::unordered_set<uint32_t> noSignalSats {40749, 40549, 40938, 40748};
    for (const auto& [satcatId, satInfo] : sats)
    {
        EXPECT_TRUE(satInfo.active);
        EXPECT_FALSE(satInfo.name.empty());
        EXPECT_FALSE(satInfo.prn.empty());
        EXPECT_EQ(satcatId, satInfo.satcat_id);
        EXPECT_EQ(time, satInfo.stamp);
        if (noSignalSats.find(satcatId) == noSignalSats.end())
        {
            EXPECT_FALSE(satInfo.signals.empty());
            for (const auto& signal : satInfo.signals)
            {
                EXPECT_EQ(signal.constellation, satInfo.constellation);
                EXPECT_NE(signal.frequency, 0.0);
                EXPECT_FALSE(signal.name.empty());
            }
        }
    }

    const auto& gpsSat = sats.at(44506);
    EXPECT_EQ(gpsSat.constellation, Enums::CONSTELLATION_GPS);
    EXPECT_EQ(gpsSat.name, "NAVSTAR 78");
    EXPECT_EQ(gpsSat.prn, "G18");
    ASSERT_EQ(gpsSat.signals.size(), 8);
    EXPECT_EQ(gpsSat.signals[0].name, Enums::SIGNAL_GPS_L1_CA);
    EXPECT_FLOAT_EQ(gpsSat.signals[0].frequency, 1575.42);
    EXPECT_EQ(gpsSat.signals[1].name, Enums::SIGNAL_GPS_L1P);
    EXPECT_FLOAT_EQ(gpsSat.signals[1].frequency, 1575.42);
    EXPECT_EQ(gpsSat.signals[2].name, Enums::SIGNAL_GPS_L1M);
    EXPECT_FLOAT_EQ(gpsSat.signals[2].frequency, 1575.42);
    EXPECT_EQ(gpsSat.signals[3].name, Enums::SIGNAL_GPS_L1C);
    EXPECT_FLOAT_EQ(gpsSat.signals[3].frequency, 1575.42);
    EXPECT_EQ(gpsSat.signals[4].name, Enums::SIGNAL_GPS_L2P);
    EXPECT_FLOAT_EQ(gpsSat.signals[4].frequency, 1277.6);
    EXPECT_EQ(gpsSat.signals[5].name, Enums::SIGNAL_GPS_L2C);
    EXPECT_FLOAT_EQ(gpsSat.signals[5].frequency, 1277.6);
    EXPECT_EQ(gpsSat.signals[6].name, Enums::SIGNAL_GPS_L2M);
    EXPECT_FLOAT_EQ(gpsSat.signals[6].frequency, 1277.6);
    EXPECT_EQ(gpsSat.signals[7].name, Enums::SIGNAL_GPS_L5);
    EXPECT_FLOAT_EQ(gpsSat.signals[7].frequency, 1176.45);

    const auto& galSat = sats.at(49809);
    EXPECT_EQ(galSat.constellation, Enums::CONSTELLATION_GALILEO);
    EXPECT_EQ(galSat.name, "GALILEO 27 (223)");
    EXPECT_EQ(galSat.prn, "E34");
    ASSERT_EQ(galSat.signals.size(), 4);
    EXPECT_EQ(galSat.signals[0].name, Enums::SIGNAL_GALILEO_E1);
    EXPECT_FLOAT_EQ(galSat.signals[0].frequency, 1575.42);
    EXPECT_EQ(galSat.signals[1].name, Enums::SIGNAL_GALILEO_E5A);
    EXPECT_FLOAT_EQ(galSat.signals[1].frequency, 1176.45);
    EXPECT_EQ(galSat.signals[2].name, Enums::SIGNAL_GALILEO_E5B);
    EXPECT_FLOAT_EQ(galSat.signals[2].frequency, 1207.14);
    EXPECT_EQ(galSat.signals[3].name, Enums::SIGNAL_GALILEO_E6);
    EXPECT_FLOAT_EQ(galSat.signals[3].frequency, 1278.75);

    const auto& gloSat = sats.at(54031);
    EXPECT_EQ(gloSat.constellation, Enums::CONSTELLATION_GLONASS);
    EXPECT_EQ(gloSat.name, "COSMOS 2559");
    EXPECT_EQ(gloSat.prn, "R25");
    ASSERT_EQ(gloSat.signals.size(), 7);
    EXPECT_EQ(gloSat.signals[0].name, Enums::SIGNAL_GLONASS_L1_CA);
    EXPECT_FLOAT_EQ(gloSat.signals[0].frequency, 1599.1875);
    EXPECT_EQ(gloSat.signals[1].name, Enums::SIGNAL_GLONASS_L1P);
    EXPECT_FLOAT_EQ(gloSat.signals[1].frequency, 1599.1875);
    EXPECT_EQ(gloSat.signals[2].name, Enums::SIGNAL_GLONASS_L2_CA);
    EXPECT_FLOAT_EQ(gloSat.signals[2].frequency, 1243.8125);
    EXPECT_EQ(gloSat.signals[3].name, Enums::SIGNAL_GLONASS_L2P);
    EXPECT_FLOAT_EQ(gloSat.signals[3].frequency, 1243.8125);
    EXPECT_EQ(gloSat.signals[4].name, Enums::SIGNAL_GLONASS_L2OC);
    EXPECT_FLOAT_EQ(gloSat.signals[4].frequency, 1248.0601);
    EXPECT_EQ(gloSat.signals[5].name, Enums::SIGNAL_GLONASS_L2SC);
    EXPECT_FLOAT_EQ(gloSat.signals[5].frequency, 1248.0601);
    EXPECT_EQ(gloSat.signals[6].name, Enums::SIGNAL_GLONASS_L3OC);
    EXPECT_FLOAT_EQ(gloSat.signals[6].frequency, 1202.025);

    const auto& beiSat = sats.at(43706);
    EXPECT_EQ(beiSat.constellation, Enums::CONSTELLATION_BEIDOU);
    EXPECT_EQ(beiSat.name, "BEIDOU 3M17");
    EXPECT_EQ(beiSat.prn, "C36");
    ASSERT_EQ(beiSat.signals.size(), 9);
    EXPECT_EQ(beiSat.signals[0].name, Enums::SIGNAL_BEIDOU_B1I);
    EXPECT_FLOAT_EQ(beiSat.signals[0].frequency, 1561.098);
    EXPECT_EQ(beiSat.signals[1].name, Enums::SIGNAL_BEIDOU_B1Q);
    EXPECT_FLOAT_EQ(beiSat.signals[1].frequency, 1561.098);
    EXPECT_EQ(beiSat.signals[2].name, Enums::SIGNAL_BEIDOU_B1C);
    EXPECT_FLOAT_EQ(beiSat.signals[2].frequency, 1575.42);
    EXPECT_EQ(beiSat.signals[3].name, Enums::SIGNAL_BEIDOU_B1A);
    EXPECT_FLOAT_EQ(beiSat.signals[3].frequency, 1575.42);
    EXPECT_EQ(beiSat.signals[4].name, Enums::SIGNAL_BEIDOU_B2A);
    EXPECT_FLOAT_EQ(beiSat.signals[4].frequency, 1176.45);
    EXPECT_EQ(beiSat.signals[5].name, Enums::SIGNAL_BEIDOU_B2B);
    EXPECT_FLOAT_EQ(beiSat.signals[5].frequency, 1207.14);
    EXPECT_EQ(beiSat.signals[6].name, Enums::SIGNAL_BEIDOU_B3I);
    EXPECT_FLOAT_EQ(beiSat.signals[6].frequency, 1268.52);
    EXPECT_EQ(beiSat.signals[7].name, Enums::SIGNAL_BEIDOU_B3Q);
    EXPECT_FLOAT_EQ(beiSat.signals[7].frequency, 1268.52);
    EXPECT_EQ(beiSat.signals[8].name, Enums::SIGNAL_BEIDOU_B3A);
    EXPECT_FLOAT_EQ(beiSat.signals[8].frequency, 1268.52);
}

TEST(IgsSatelliteMetadata, LoadConstellation)  // NOLINT
{
    IGSSatelliteMetadata metadata;
    ASSERT_TRUE(metadata.load());

    ros::Time time {1699955783, 0};
    const auto sats = metadata.getSatellites(time, true, {Enums::CONSTELLATION_GALILEO});
    ASSERT_EQ(sats.size(), 28u);

    for (const auto& [satcatId, satInfo] : sats)
    {
        EXPECT_TRUE(satInfo.active);
        EXPECT_EQ(satInfo.constellation, Enums::CONSTELLATION_GALILEO);
        EXPECT_FALSE(satInfo.name.empty());
        EXPECT_FALSE(satInfo.prn.empty());
        EXPECT_EQ(satcatId, satInfo.satcat_id);
        EXPECT_EQ(time, satInfo.stamp);
        EXPECT_FALSE(satInfo.signals.empty());
        for (const auto& signal : satInfo.signals)
        {
            EXPECT_EQ(signal.constellation, satInfo.constellation);
            EXPECT_NE(signal.frequency, 0.0);
            EXPECT_FALSE(signal.name.empty());
        }
    }

    ASSERT_NE(sats.find(49809), sats.end());
    const auto& galSat = sats.at(49809);
    EXPECT_EQ(galSat.constellation, Enums::CONSTELLATION_GALILEO);
    EXPECT_EQ(galSat.name, "GALILEO 27 (223)");
    EXPECT_EQ(galSat.prn, "E34");
    ASSERT_EQ(galSat.signals.size(), 4);
    EXPECT_EQ(galSat.signals[0].name, Enums::SIGNAL_GALILEO_E1);
    EXPECT_FLOAT_EQ(galSat.signals[0].frequency, 1575.42);
    EXPECT_EQ(galSat.signals[1].name, Enums::SIGNAL_GALILEO_E5A);
    EXPECT_FLOAT_EQ(galSat.signals[1].frequency, 1176.45);
    EXPECT_EQ(galSat.signals[2].name, Enums::SIGNAL_GALILEO_E5B);
    EXPECT_FLOAT_EQ(galSat.signals[2].frequency, 1207.14);
    EXPECT_EQ(galSat.signals[3].name, Enums::SIGNAL_GALILEO_E6);
    EXPECT_FLOAT_EQ(galSat.signals[3].frequency, 1278.75);
}

TEST(IgsSatelliteMetadata, LoadSignal)  // NOLINT
{
    IGSSatelliteMetadata metadata;
    ASSERT_TRUE(metadata.load());

    ros::Time time {1699955783, 0};
    const auto sats = metadata.getSatellites(time, true, {}, {Enums::SIGNAL_GPS_L5});
    ASSERT_EQ(sats.size(), 18u);

    for (const auto& [satcatId, satInfo] : sats)
    {
        EXPECT_TRUE(satInfo.active);
        EXPECT_EQ(satInfo.constellation, Enums::CONSTELLATION_GPS);
        EXPECT_FALSE(satInfo.name.empty());
        EXPECT_FALSE(satInfo.prn.empty());
        EXPECT_EQ(satcatId, satInfo.satcat_id);
        EXPECT_EQ(time, satInfo.stamp);
        ASSERT_EQ(satInfo.signals.size(), 1u);
        EXPECT_EQ(satInfo.signals[0].constellation, satInfo.constellation);
        EXPECT_NE(satInfo.signals[0].frequency, 0.0);
        EXPECT_FALSE(satInfo.signals[0].name.empty());
    }

    ASSERT_NE(sats.find(44506), sats.end());
    const auto& gpsSat = sats.at(44506);
    EXPECT_EQ(gpsSat.constellation, Enums::CONSTELLATION_GPS);
    EXPECT_EQ(gpsSat.name, "NAVSTAR 78");
    EXPECT_EQ(gpsSat.prn, "G18");
    ASSERT_EQ(gpsSat.signals.size(), 1);
    EXPECT_EQ(gpsSat.signals[0].name, Enums::SIGNAL_GPS_L5);
    EXPECT_FLOAT_EQ(gpsSat.signals[0].frequency, 1176.45);
}

int main(int argc, char** argv)
{
    setenv("GNSS_INFO_CACHE_DIR", TEST_CACHE_DIR, true);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
