# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# ECEF position and velocity of a satellite.

uint32 satcat_id  # SATCAT number (NORAD ID) of the satellite.
geometry_msgs/Point position  # Position of the satellite in ECEF frame.
geometry_msgs/Vector3 velocity  # Velocity of the satellite in ECEF frame.
float64[9] position_covariance  # Row-major covariance matrix of position. If all zeros, it is not specified.
float64[9] velocity_covariance  # Row-major covariance matrix of velocity. If all zeros, it is not specified.