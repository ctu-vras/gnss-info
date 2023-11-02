# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Position of a satellite in the sky as seen from a particular reference position.
# This message should not be used standalone, but only as a part of SkyView message.

uint32 satcat_id  # SATCAT number (NORAD ID) of the satellite.
geometry_msgs/Point position  # Position of the satellite in ECEF frame.
geometry_msgs/Vector3 velocity  # Velocity of the satellite in ECEF frame.
float32 azimuth_deg  # Azimuth in the sky (in degress, NED convention).
float32 elevation_deg  # Elevation in the sky (in degress, positive values above local horizon).
float64 distance  # Distance of the satellite to the reference position (corrected for Sagnac effect; in meters).