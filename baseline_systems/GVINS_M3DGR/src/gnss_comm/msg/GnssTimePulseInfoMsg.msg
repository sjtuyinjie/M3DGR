# This message contains information of UBX-TIM-TP message. 
# reference: [1]. UBX-18010854-R08, page 156

GnssTimeMsg time    # GNSS time of the next time pulse
bool utc_based      # if the time is UTC (1=UTC, 0=GNSS)
uint32 time_sys     # GNSS time reference system, if the time base is GNSS (utc_base=0)