# Range Information

# Satellite PRN/Slot 
int16 prn_slot

# Pseudorange measurement (m)
float64 psr

# Pseudorange measurement standard deviation (m)
float32 psr_std

# Carrier phase, in cycles (accumulated Doppler range)
float64 adr

# Estimated carrier phase standard deviation (cycles)
float32 adr_std

# Instantaneous carrier Doppler frequency (Hz)
float32 doppler

# Carrier to noise density ratio C/No = 10[log10(S/N0)] (dB-Hz) (db-Hz)
float32 c_no

# Number of seconds of continuous tracking (no cycle slipping)
float32 locktime

# Channel tracking status
uint32 ch_tr_status

# Tracking Status
TrackingStatus tracking_status
