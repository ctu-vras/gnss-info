# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Information about orbital elements of a satellite (mostly corresponds to TLE format).

time epoch  # Epoch (time around which the parameters are valid).
uint32 satcat_id  # SATCAT number (NORAD ID) of the satellite.
float32 mean_motion_dot  # 1st derivative of mean motion
float32 mean_motion_ddot  # 2nd derivative of mean motion
float32 b_star  # B* or drag term
float32 inclination_deg  # Inclination of the satellite's orbit plane relative to equator (in degrees).
float32 right_ascension_deg  # Right ascension of the satellite orbit's ascending node (in degrees).
float32 eccentricity  # Eccentricity of the orbit.
float32 argument_of_perigee_deg  # Argument of perigee (in degrees).
float32 mean_anomaly_deg  # Mean anomaly (in degrees).
float32 mean_motion  # Mean motion (in revolutions/day).
uint32 revolution_number  # Revolution number at epoch.