
# 0 SOL_COMPUTED Solution computed
# 1 INSUFFICIENT_OBS Insufficient observations
# 2 NO_CONVERGENCE No convergence
# 3 SINGULARITY Singularity at parametres matrix
# 4 COV_TRACE Covariance trace exceeds maximum (trace > 1000 m)
# 5 TEST_DIST Test distance exceeded (maximum of 3 rejections if distance > 10 km)
# 6 COLD_START Not yet converged from cold start
# 7 V_H_LIMIT Height or velocity limits exceeded (in accordance with export licensing restrictions)
# 8 VARIANCE Variance exceeds limits
# 9 RESIDUALS Residuals are too large
# 10 DELTA_POS Delta position is too large
# 11 NEGATIVE_VAR Negative variance
# 13 INTEGRITY_WARNING Large residuals make position unreliable
# 18 PENDING When a FIX POSITION command is entered, the receiver computes its own position and determines if the fixed position is valid
# 19 INVALID_FIX The fixed position, entered using the FIX POSITION command, is not valid
# 20 UNAUTHORIZED Position type is unauthorized - HP or XP on a receiver not authorized for it
# 21 ANTENNA_WARNING One of the antenna warnings listed in the RTKANTENNA command description, see page 172

uint32 SOL_COMPUTED       = 0
uint32 INSUFFICIENT_OBS   = 1
uint32 NO_CONVERGENCE     = 2
uint32 SINGULARITY        = 3
uint32 COV_TRACE          = 4
uint32 TEST_DIST          = 5
uint32 COLD_START         = 6
uint32 V_H_LIMIT          = 7
uint32 VARIANCE           = 8
uint32 RESIDUALS          = 9
uint32 DELTA_POS          = 10
uint32 NEGATIVE_VAR       = 11
uint32 INTEGRITY_WARNING  = 13
uint32 PENDING            = 18
uint32 INVALID_FIX        = 19
uint32 UNAUTHORIZED       = 20
uint32 ANTENNA_WARNING    = 21

uint32 solution_status
