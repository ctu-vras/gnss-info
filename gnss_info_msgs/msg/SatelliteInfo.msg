# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# General information about a satellite.

time stamp  # The time to which this data are valid. Some values like prn or active state may change over time.
uint32 satcat_id  # SATCAT number (NORAD ID) of the satellite. This ID should be used as a stable identifier.
string name  # Name of the satellite. It should not be used as a stable identifier.
string prn  # PRN of the satellite at the given time. If the satellite is not active at the given time, empty string.
            # Either 2-3 digits, or 1 letter and 2 digits.
string constellation  # One of CONSTELLATION_* values from Enums.msg
bool active  # True if the satellite is in operational state, False otherwise (decommissioned, faulty, experimental).
SatelliteSignal[] signals  # The signal types transmitted by the satellite at the given time.