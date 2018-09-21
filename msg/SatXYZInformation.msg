# Information about SV position in ECEF Cartesion Coordinates

# Satellite PRN/Slot 
int16 prn_slot

# Satellite XYZ coordinates (ECEF, m)
geometry_msgs/Point position

# Satellite Clock correction (m)
float64 clk_corr
# Satellite Ionospheric correction (m)
float64 ion_corr
# Satellite Tropospheric correction (m)
float64 trop_corr
