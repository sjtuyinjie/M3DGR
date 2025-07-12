# This message contains one-epoch measurements from one single satellite

GnssTimeMsg time    # measurement time
uint32 sat          # satellite ID (define by `sat_no` function in `gnss_utility.hpp`)
float64[] freqs     # observed frequencies [Hz]
float64[] CN0       # carrier-to-noise density ratio (signal strength) [dB-Hz]
uint8[] LLI         # lost-lock indicator (1=lost)
uint8[] code        # channel code
float64[] psr       # pseudorange measurement [m]
float64[] psr_std   # pseudorange standard deviation [m]
float64[] cp        # carrier phase measurement [cycle]
float64[] cp_std    # carrier phase standard deviation [cycle]
float64[] dopp      # Doppler measurement [Hz]
float64[] dopp_std  # Doppler standard deviation [Hz]
# tracking status. bit_0:psr valid, bit_1:cp valid, 
# bit_2:half cp valid, bit_3:half cp subtracted
uint8[]   status    