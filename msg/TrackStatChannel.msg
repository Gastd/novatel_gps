# Tracking Status for a single hardware channel

# Satellite PRN/Slot 
int16 prn_slot

# Channel tracking status
uint32 ch_tr_status

# Tracking Status
TrackingStatus tracking_status

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
