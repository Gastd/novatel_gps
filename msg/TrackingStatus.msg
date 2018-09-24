
# Tracking State Codes
# 0 L1 Idle
# 1 L1 Sky search
# 2 L1 Wide frequency band pull-in
# 3 L1 Narrow frequency band pull-in
# 4 L1 Phase lock loop
# 5 L1 Reacquisition
# 6 L1 Steering
# 7 L1 Frequency-lock loop

uint16 L1_IDLE = 0
uint16 L1_SKY_SEARCH= 1
uint16 L1_WIDE_PULL_IN = 2
uint16 L1_NARROW_PULL_IN = 3
uint16 L1_PLL = 4
uint16 L1_REACQUISITION = 5
uint16 L1_STEERING = 6
uint16 L1_FLL = 7

# Tracking State
uint16 trck_state

# SV channel number
uint16 channel_number

# Phase lock flag
bool phase_lock

# Parity known flag
bool parity_known

# Code lock flag
bool code_lock

# Correlator Type Codes
# 0 N/A
# 1 Standard correlator: spacing = 1 chip
# 2 Narrow Correlator: spacing < 1 chip
# 4 Pulse Aperture Correlator (PAC)

uint16 CORRELATOR_NA = 0
uint16 STANDARD_CORRELATOR = 1
uint16 NARROW_CORRELATOR = 2
uint16 PAC = 4

# Correlator type
uint16 correlator_type

# Satellite System Codes
# 0 GPS
# 1 GLONASS
# 2 WAAS
uint8 GPS = 0
uint8 GLONASS = 1
uint8 WAAS = 2

# Satellite system
uint8 satellite_system

# Grouping
bool grouping

# Signal Type Codes
# 0 L1 C/A
uint16 L1_CA = 0
uint16 L2_P = 5
uint16 L2_P_CODELESS = 9

# Signal Type
uint16 singal_type

# Forward Error Correction
bool fec

# Primary L1 channel
bool primary_l1

# Carrier phase measurement
bool half_cycle_added

# PRN lock flag
bool prn_lock

# Channel assignment
bool channel_assignment
