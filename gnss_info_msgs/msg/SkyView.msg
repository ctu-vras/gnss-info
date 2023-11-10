# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# View of the satellites in the sky from a particular location on Earth. It can be either all available satellites,
# or only satellites visible by a particular sensor.

Header header  # Frame should be "wgs84" or any other datum in which the reference position is expressed.
geographic_msgs/GeoPoint reference_position  # The reference position from which the sky is observed.
float32 elevation_mask_deg  # Minimum elevation of satellites that are considered (in degrees).
SatelliteSkyPosition[] satellites
DOP dop