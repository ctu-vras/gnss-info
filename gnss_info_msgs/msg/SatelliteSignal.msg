# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# General information about a particular satellite signal.

string name  # Name of the signal. One of SIGNAL_* values from Enums.msg
string constellation  # Constellation the signal is a part of. One of CONSTELLATION_* values from Enums.msg
float32 frequency  # Center frequency band of the signal in Hz (0 if not known). For GLONASS, the channel-based
                   # frequency shift should be applied.