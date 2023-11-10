# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Dilution of precision. If less than 4 satellites are visible, all DOP values should be set to infinity.

float32 gdop  # Geometrical dilution of precision
float32 pdop  # Position dilution of precision
float32 hdop  # Horizontal dilution of precision
float32 vdop  # Vertical dilution of precision
float32 tdop  # Time dilution of precision