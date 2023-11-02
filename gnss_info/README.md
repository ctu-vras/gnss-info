<!--
SPDX-License-Identifier: BSD-3-Clause
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# gnss_info

Information about GNSS satellites and their constellations.

## Provided C++ libraries

- `libsatellite_metadata.so / igs_satellite_metadata.h`: General information about satellites downloaded from IGS.

## Nodes

### publish_all_satellites

An example node using IGSSatelliteMetadata to construct a list of all GNSS satellites that existed in a given 
time instant.

#### Published topics
- `satellites` (`gnss_info_msgs/SatellitesList`, latched): The list of satellites.

#### Parameters
- `~time` (float, defaults to ros::Time::now() ): The reference time for which satellites should be looked up.
- `~only_active` (bool, default true): Return only satellites active at the reference time.
- `~only_constellations` (list of string): If nonempty, limits the published satellites to only those belonging to
                                           the listed constellations.
- `~only_signals` (list of string): If nonempty, limits the published satellites to only those transmitting at least
                                    one of the listed signals.