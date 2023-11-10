<!--
SPDX-License-Identifier: BSD-3-Clause
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# gnss_info

Information about GNSS satellites and their constellations.

## Provided C++ libraries

- `libsatellite_metadata.so / igs_satellite_metadata.h`: General information about satellites downloaded from IGS.
- `liborbital_data.so / orbital_data_manager.h`: Library that manages downloads of almanac/ephemeris data to support
  computing satellite orbits. It can also generate a sky view from given place and time.

## Nodes

### publish_all_satellites

An example node using `IGSSatelliteMetadata` to construct a list of all GNSS satellites that existed in a given 
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

### publish_sky_view

An example node using `OrbitalDataManager` with `EthzSatdbProvider` to get approximate locations of satellites.

#### Published topics
- `satellites_positions` (`gnss_info_msgs/SatellitesPositions`, latched): ECEF positions of all satellites from
  `satellites` topic for which orbital data are available.
- `sky_view` (`gnss_info_msgs/SkyView`, latched): Sky view of all satellites from `satellites` topic which are
  visible from the last position received on `position` topic.

#### Subscribed topics
- `satellites` (`gnss_info_msgs/SatellitesList`): The list of satellites.
- `position` (`geographic_msgs/GeoPoint`): Reference position for the sky view.

#### Parameters
- `~elevation_mask_deg` (float, default 5.0°): Minimum elevation of satellites to be considered in the sky view.
- `~refresh_rate` (double, default 0.1 Hz): Rate at which updates of the positions should be published and recomputed.

### sky_plot

A node turning `gnss_info_msgs/SkyView` into a plot of satellites.

#### Published topics
- `sky_plot` (`sensor_msgs/Image`): The sky plot.

#### Subscribed topics
- `satellites` (`gnss_info_msgs/SatellitesList`): The list of satellites.
- `sky_view` (`gnss_info_msgs/SkyView`): Sky view of some satellites.

#### Parameters
- `~plot_color` (str, default "white"): Background color of the plot. Anything that matplotlib reads (e.g. `#ABCDEF00`).
- `~show_legend` (bool, default `True`): Whether to show legend in the plot.
- `~show_labels` (bool, default `True`): Whether to show labels of the individual satellites.
- `~show_title` (bool, default `True`): Whether to show title with plot time and reference location.
- `~show_elevation_mask` (bool, default `True`): Whether to show a greyed-out area in masked parts of the plot.
- `~width` (int, default 840): Width of the plot in px.
- `~height` (int, default 720): Height of the plot in px.
- `~dpi` (float, default 120.0): DPI of the plot.
- `~only_constellations` (str or list[str], default ""): Either a comma-separated list or a list of string defining
                                                         which constellations to plot. If empty, everything is plotted.

## Possible improvements:

- Get GPS outages from https://www.navcen.uscg.gov/sites/default/files/gps/sof/current_sof.sof .
- Get Galileo outages from NAGUs
  - https://www.gsc-europa.eu/sites/default/files/sites/all/files/UNP_UNUFN.txt
  - https://www.gsc-europa.eu/system-status/user-notifications-archived
- Precise orbits from http://ftp.aiub.unibe.ch/CODE_MGEX/CODE/2023/ (ORB.SP3 files)
  - `gnsstk/core/lib/GNSSEph/EphemerisRange.hpp`
  - `gnsstk/core/lib/Geomatics/PreciseRange.cpp`