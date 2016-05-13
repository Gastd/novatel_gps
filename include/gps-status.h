/**
 * @file      gps-status.h
 * @author    George Andrew Brindeiro
 * @date      18/10/2011
 *
 * @attention Copyright (C) 2011
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */

// Response (Firmware Reference Manual, p.629, Table 109)
#define R_OKAY          1   // Command was received correctly
#define R_LOG_DNE       2   // The log requested does not exist
#define R_NE_RSRC       3   // The request has exceeded a limit (for example, the maximum number of logs are being generated)
#define R_DATAP_NVER    4   // Data packet is not verified
#define R_COMM_FAIL     5   // Command did not succeed in accomplishing requested task
#define R_INV_ID        6   // The input message ID is not valid
#define R_INV_FIELD     7   // Field x of the input message is not correct
#define R_INV_CHKSUM    8   // The checksum of the input message is not correct (this only applies to ASCII and binary format messages)
#define R_MSNG_FIELD    9   // A field is missing from the input message
#define R_MSG_TMOUT     31  // Message timed out

// GPS Time Status (Firmware Reference Manual, p.30, Table 8)
#define T_UNKNOWN           20  // Time validity is unknown
#define T_APPROXIMATE       60  // Time is set approximately
#define T_COURSEADJUSTING   80  // Time is approaching coarse precision (+-10ms)
#define T_COARSE            100 // This time is valid to coarse precision
#define T_COARSESTEERING    120 // Time is coarse set, and is being steered
#define T_FREEWHEELING      130 // Position is lost, and the range bias cannot be calculated
#define T_FINEADJUSTING     140 // Time is adjusting to fine precision
#define T_FINE              160 // Time has fine precision (+-1us)
#define T_FINESTEERING      180 // Time is fine set and is being steered
#define T_SATTIME           200 // Time from satellite. This is only used in logs containing satellite data such as ephemeris and almanac

// GPS Status

// Position/Velocity Status (Firmware Reference Manual, p.253, Table 51)
#define PV_SOL_COMPUTED         0   // Solution computed
#define PV_INSUFFICIENT_OBS     1   // Insufficient observations
#define PV_NO_CONVERGENCE       2   // No convergence
#define PV_SINGULARITY          3   // Singularity at parametres matrix
#define PV_COV_TRACE            4   // Covariance trace exceeds maximum (trace > 1000 m)
#define PV_TEST_DIST            5   // Test distance exceeded (maximum of 3 rejections if distance > 10 km)
#define PV_COLD_START           6   // Not yet converged from cold start
#define PV_V_H_LIMIT            7   // Height or velocity limits exceeded (in accordance with export licensing restrictions)
#define PV_VARIANCE             8   // Variance exceeds limits
#define PV_RESIDUALS            9   // Residuals are too large
#define PV_DELTA_POS            10  // Delta position is too large
#define PV_NEGATIVE_VAR         11  // Negative variance
#define PV_INTEGRITY_WARNING    13  // Large residuals make position unreliable
#define PV_PENDING              18  // When a FIX POSITION command is entered, the receiver computes its own position and determines if the fixed position is valid
#define PV_INVALID_FIX          19  // The fixed position, entered using the FIX POSITION command, is not valid
#define PV_ANTENNA_WARNING      21  // One of the antenna warnings listed in the RTKANTENNA command description, see page 172

