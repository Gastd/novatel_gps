# 20 UNKNOWN Time validity is unknown.
# 60 APPROXIMATE Time is set approximately.
# 80 COARSEADJUSTING Time is approaching coarse precision.
# 100 COARSE This time is valid to coarse precision.
# 120 COARSESTEERING Time is coarse set, and is being steered.
# 130 FREEWHEELING Position is lost, and the range bias cannot be calculated.
# 140 FINEADJUSTING Time is adjusting to fine precision.
# 160 FINE Time has fine precision.
# 180 FINESTEERING Time is fine set and is being steered.
# 200 SATTIME Time from satellite. This is only used in logs containing satellite data such as ephemeris and almanac


uint8 UNKNOWN            = 20
uint8 APPROXIMATE        = 60
uint8 COARSEADJUSTING    = 80
uint8 COARSE             = 100
uint8 COARSESTEERING     = 120
uint8 FREEWHEELING       = 130
uint8 FINEADJUSTING      = 140
uint8 FINE               = 160
uint8 FINESTEERING       = 180
uint8 SATTIME            = 200

uint8 time_stat
