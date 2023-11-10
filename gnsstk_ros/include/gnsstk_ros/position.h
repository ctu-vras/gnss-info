// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <gnsstk/Position.hpp>
#include <gnsstk/Xvt.hpp>

#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Point.h>
#include <gnss_info_msgs/SatellitePosition.h>

namespace gnsstk_ros
{

/**
 * \brief Convert the ROS GeoPoint message to gnsstk Position with Geodetic type.
 * \param[in] position The WGS84 position to convert.
 * \return The corresponding gnsstk position object.
 */
gnsstk::Position convert(const geographic_msgs::GeoPoint& position);

/**
 * \brief Convert the given gnsstk Position object to WGS84 ROS GeoPoint message.
 * \param[in] position The gnsstk position to convert.
 * \return The corresponding ROS GeoPoint.
 */
geographic_msgs::GeoPoint convertToGeographicMsg(const gnsstk::Position& position);

/**
 * \brief Convert the ROS Point ECEF position to gnsstk Position with Cartesian type.
 * \param[in] position The ECEF position to convert.
 * \return The corresponding gnsstk position object.
 */
gnsstk::Position convert(const geometry_msgs::Point& position);

/**
 * \brief Convert the given gnsstk Position object to ECEF ROS Point message.
 * \param[in] position The gnsstk position to convert.
 * \return The corresponding ROS Point in ECEF coordinates.
 */
geometry_msgs::Point convertToCartesianMsg(const gnsstk::Position& position);

/**
 * \brief Convert the given position and velocity to ROS SatellitePosition message.
 * \param[in] xvt The position and velocity as determined by gnsstk.
 * \param[in] satcatId ID of the satellite in the satellite catalog.
 * \return The ROS SatellitePosition message corresponding to the given parameter.
 */
gnss_info_msgs::SatellitePosition convert(const gnsstk::Xvt& xvt, uint32_t satcatId);

}
