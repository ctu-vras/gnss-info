// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <gnsstk/Position.hpp>

#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Point.h>
#include <gnsstk_ros/position.h>

namespace gnsstk_ros
{

gnsstk::Position convert(const geographic_msgs::GeoPoint& position)
{
    return {position.latitude, position.longitude, position.altitude, gnsstk::Position::CoordinateSystem::Geodetic};
}

geographic_msgs::GeoPoint convertToGeographicMsg(const gnsstk::Position& position)
{
    auto pos = position;
    pos.transformTo(gnsstk::Position::CoordinateSystem::Geodetic);
    geographic_msgs::GeoPoint msg;
    msg.latitude = pos.geodeticLatitude();
    msg.longitude = pos.longitude();
    msg.altitude = pos.height();
    return msg;
}

gnsstk::Position convert(const geometry_msgs::Point& position)
{
    return {position.x, position.y, position.z, gnsstk::Position::CoordinateSystem::Cartesian};
}

geometry_msgs::Point convertToCartesianMsg(const gnsstk::Position& position)
{
    auto pos = position;
    pos.transformTo(gnsstk::Position::CoordinateSystem::Cartesian);
    geometry_msgs::Point msg;
    msg.x = pos.X();
    msg.y = pos.Y();
    msg.z = pos.Z();
    return msg;
}

gnss_info_msgs::SatellitePosition convert(const gnsstk::Xvt& xvt, const uint32_t satcatId,
    const double posCov, const double velCov)
{
    gnss_info_msgs::SatellitePosition position;
    position.satcat_id = satcatId;
    position.position.x = xvt.x[0];
    position.position.y = xvt.x[1];
    position.position.z = xvt.x[2];
    position.velocity.x = xvt.v[0];
    position.velocity.y = xvt.v[1];
    position.velocity.z = xvt.v[2];
    position.position_covariance = {posCov, 0.0, 0.0, 0.0, posCov, 0.0, 0.0, 0.0, posCov};
    position.velocity_covariance = {velCov, 0.0, 0.0, 0.0, velCov, 0.0, 0.0, 0.0, velCov};
    return position;
}

}
