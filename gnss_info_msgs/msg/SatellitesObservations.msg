# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Observation of satellites in the sky by a particular sensor.

Header header  # Frame should be "wgs84" or any other datum in which the reference position is expressed.
               # Stamp is receiver time.
float32 elevation_mask_deg  # Minimum elevation of satellites that are considered (in degrees).
SatelliteObservation[] observations