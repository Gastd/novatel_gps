# Tracking Status for a single hardware channel

# Satellite PRN/Slot 
int16 prn_slot

# Channel tracking status
uint32 ch_tr_status

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

# Pseudorange (m)
float64 psr

# Doppler frequency (Hz)
float32 doppler

# Carrier to noise density ratio (dB-Hz)
float32 cn0

# Number of seconds of continuous tracking (no cycle slips) 
float32 locktime

# Pseudorange residual from pseudorange filter
float32 psr_res

# Reject Code
# GOOD = 0                  Observation is good
# BADHEALTH = 1             Bad satellite health is indicated by ephemeris data
# OLDEPHEMERIS = 2          Old ephemeris due not being updated during the last 3 hours
# ECCENTRICANOMALY = 3      Eccentric anomaly error during computation of the satellite’s position
# TRUEANOMALY = 4           True anomaly error during computation of the satellite’s position
# SATCOORDINATE_ERROR = 5   Satellite coordinate error during computation of the satellite’s position
# ELEVATIONERROR = 6        Elevation error due to the satellite being below the cut-off angle
# MISCLOSURE = 7            Misclosure too large due to excessive gap between estimated and actual positions
# NODIFFCORR = 8            No compatible differential correction is available for this particular satellite
# NOEPHEMERIS = 9           Ephemeris data for this satellite has not yet been received
# INVALIDIODE = 10          Invalid IODE (Issue Of Data Ephemeris) due to mismatch between differential stations
# LOCKEDOUT = 11            Locked out: satellite is excluded by the user (LOCKOUT command)
# LOWPOWER = 12             Low power: satellite is rejected due to low carrier/noise ratio
# OBSL2                     L2 observation is ignored and not used in the pseudorange solution
# NOIONOCORR                No compatible ionospheric correction is available for this particular satellite
# NOTUSED                   Observation is ignored and not used in the solution
# NA                        No observation (a reject code is not applicable)
# BAD_INTEGRITY             The integrity of the pseudorange is bad

uint32 GOOD = 0
uint32 BADHEALTH = 1
uint32 OLDEPHEMERIS = 2
uint32 ECCENTRICANOMALY = 3
uint32 TRUEANOMALY = 4
uint32 SATCOORDINATE_ERROR = 5
uint32 ELEVATIONERROR = 6
uint32 MISCLOSURE = 7
uint32 NODIFFCORR = 8
uint32 NOEPHEMERIS = 9
uint32 INVALIDIODE = 10
uint32 LOCKEDOUT = 11
uint32 LOWPOWER = 12
uint32 OBSL2 = 13
uint32 NOIONOCORR = 16
uint32 NOTUSED = 16
uint32 REJECT_NA = 99
uint32 BAD_INTEGRITY = 100

# Range reject code from pseudorange filter
uint32 reject

# Pseudorange filter weighting
float32 psr_weight
