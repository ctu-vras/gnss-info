# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

# Enumerations defining expected values of some fields in the other messages.

# Extracting the enums to this empty message allows for evolving the enums without changing the
# MD5 sum of the data messages.
# To ensure backwards compatibility, no values from these enums will be deleted in the future,
# and integral enum values will never be re-numbered with a different meaning.

# Beware that definition of string constants should not contain any quotes and also any comments on the same line!

uint16 GNSS_INFO_ENUMS_VERSION = 1  # This version will be increased every time a change is done to the defined enums.

# Name of a satellite constellation
string CONSTELLATION_GPS = GPS
string CONSTELLATION_GALILEO = GALILEO
string CONSTELLATION_GLONASS = GLONASS
string CONSTELLATION_BEIDOU = BEIDOU
string CONSTELLATION_QZSS = QZSS
# Alternatively: IRNSS
string CONSTELLATION_NAVIC = NAVIC
string CONSTELLATION_WAAS = WAAS
string CONSTELLATION_EGNOS = EGNOS
string CONSTELLATION_MSAS = MSAS
string CONSTELLATION_GAGAN = GAGAN
string CONSTELLATION_SDCM = SDCM
string CONSTELLATION_SOUTHPAN = SOUTHPAN
string CONSTELLATION_WAGE = WAGE
string CONSTELLATION_STARFIRE = STARFIRE
string CONSTELLATION_STARFIX = STARFIX
string CONSTELLATION_OMNISTAR = OMNISTAR
string CONSTELLATION_ATLAS = ATLAS

# Strings uniquely identifying various satellite signals.
# https://gssc.esa.int/navipedia/index.php/GPS_Signal_Plan etc. or https://files.igs.org/pub/data/format/rinex_4.01.pdf

## GPS ##
# L1 C/A signal
string SIGNAL_GPS_L1_CA = GPS-L1-CA
# Newer L1C civilian signal
string SIGNAL_GPS_L1C = GPS-L1C
# L1 P(Y) precision signal
string SIGNAL_GPS_L1P = GPS-L1P
# L1M military signal
string SIGNAL_GPS_L1M = GPS-L1M
# Newer L2C civilian signal
string SIGNAL_GPS_L2C = GPS-L2C
# L2 P(Y) precision signal
string SIGNAL_GPS_L2P = GPS-L2P
# L2M military signal
string SIGNAL_GPS_L2M = GPS-L2M
string SIGNAL_GPS_L5 = GPS-L5

## GALILEO ##
string SIGNAL_GALILEO_E1 = GALILEO-E1
string SIGNAL_GALILEO_E5A = GALILEO-E5A
string SIGNAL_GALILEO_E5B = GALILEO-E5B
string SIGNAL_GALILEO_E6 = GALILEO-E6

## GLONASS ##
# L1 FDMA C/A signal (ST), also called L1OF
string SIGNAL_GLONASS_L1_CA = GLONASS-L1-CA
# L1 FDMA Obfuscated signal (VT), also called L1SF
string SIGNAL_GLONASS_L1P = GLONASS-L1P
# L1 CDMA Open signal
string SIGNAL_GLONASS_L1OC = GLONASS-L1OC
# L1 CDMA Obfuscated signal
string SIGNAL_GLONASS_L1SC = GLONASS-L1SC
# L1 CDMA interoperability with L1C/E1/B1C
string SIGNAL_GLONASS_L1OCM = GLONASS-L1OCM
# L2 FDMA C/A signal (ST), also called L2OF
string SIGNAL_GLONASS_L2_CA = GLONASS-L2-CA
# L2 FDMA Obfuscated signal (VT), also called L2SF
string SIGNAL_GLONASS_L2P = GLONASS-L2P
# L2 CDMA Open signal
string SIGNAL_GLONASS_L2OC = GLONASS-L2OC
# L2 CDMA Obfuscated signal
string SIGNAL_GLONASS_L2SC = GLONASS-L2SC
# L3 CDMA Open signal
string SIGNAL_GLONASS_L3OC = GLONASS-L3OC
# L3 CDMA Obfuscated signal
string SIGNAL_GLONASS_L3SC = GLONASS-L3SC
# CDMA interoperability with E5b/B2b
string SIGNAL_GLONASS_L3OCM = GLONASS-L3OCM
# CDMA interoperability with L5/E5a/B2a
string SIGNAL_GLONASS_L5OCM = GLONASS-L5OCM

## BEIDOU ##
# Legacy Beidou-1 S-band signal
string SIGNAL_BEIDOU_BD1 = BEIDOU-BD1
# B1I public signal
string SIGNAL_BEIDOU_B1I = BEIDOU-B1I
# B1Q restricted signal
string SIGNAL_BEIDOU_B1Q = BEIDOU-B1Q
# B1C public signal
string SIGNAL_BEIDOU_B1C = BEIDOU-B1C
# B1C SBAS signal
string SIGNAL_BEIDOU_B1C_SBAS = BEIDOU-B1C-SBAS
# B1C authorized signal
string SIGNAL_BEIDOU_B1A = BEIDOU-B1A
# B2I public signal
string SIGNAL_BEIDOU_B2I = BEIDOU-B2I
# B2Q restricted signal
string SIGNAL_BEIDOU_B2Q = BEIDOU-B2Q
# B2a public signal
string SIGNAL_BEIDOU_B2A = BEIDOU-B2A
# B2a SBAS signal
string SIGNAL_BEIDOU_B2A_SBAS = BEIDOU-B2A-SBAS
# B2b public signal
string SIGNAL_BEIDOU_B2B = BEIDOU-B2B
# B2b PPP signal
string SIGNAL_BEIDOU_B2B_PPP = BEIDOU-B2B-PPP
# B3I public signal
string SIGNAL_BEIDOU_B3I = BEIDOU-B3I
# B3Q restricted signal
string SIGNAL_BEIDOU_B3Q = BEIDOU-B3Q
# B3A authorized signal
string SIGNAL_BEIDOU_B3A = BEIDOU-B3A

## QZSS ##
string SIGNAL_QZSS_L1 = QZSS-L1
string SIGNAL_QZSS_L2 = QZSS-L2
string SIGNAL_QZSS_L5 = QZSS-L5
string SIGNAL_QZSS_L6 = QZSS-L6

## NAVIC ##
string SIGNAL_NAVIC_L1 = NAVIC-L1
string SIGNAL_NAVIC_L5 = NAVIC-L5