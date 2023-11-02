# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Observation of a single satellite in the sky by a particular sensor. This message is intended to be used only inside
# SatellitesObservation message.

SatelliteSignal signal
float64 pseudorange  # The measured pseudorange (NaN if not measured; in meters).
float64 carrier_phase  # The estimated carrier phase (NaN if not measured; float number of cycles).
float64 doppler_shift  # Difference between observed and emitted frequency of the carrier (NaN if not measured; in Hz).
float64 carrier_to_noise_ratio  #  Strength of the received signal (NaN if not measured; in dbHz).
bool carrier_locked  # False if carrier lock has been lost since the last measurement (thus a cycle slip is possible).