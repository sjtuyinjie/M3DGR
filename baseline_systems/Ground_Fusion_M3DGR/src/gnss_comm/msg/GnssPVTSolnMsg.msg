# This message contains information of UBX-NAV-PVT message. 
# reference: [1]. UBX-18010854-R08, page 132

GnssTimeMsg time    # GNSS time of the navigation epoch
# GNSS fix type (0=no fix, 1=dead reckoning only, 2=2D-fix, 3=3D-fix, 
# 4=GNSS+dead reckoning combined, 5=time only fix)
uint8   fix_type
bool    valid_fix   # if fix valid (1=valid fix)
bool    diff_soln   # if differential correction were applied (1=applied)
uint8   carr_soln   # carrier phase range solution status (0=no carrier phase, 1=float, 2=fix)
uint8   num_sv      # number of satellites used in the solution
float64 latitude    # latitude [degree]
float64 longitude   # longitude [degree]
float64 altitude    # height above ellipsoid [m]
float64 height_msl  # height above mean sea level [m]
float64 h_acc       # horizontal accuracy estimate [m]
float64 v_acc       # vertical accuracy estimate [m]
float64 p_dop       # Position DOP
float64 vel_n       # NED north velocity [m/s]
float64 vel_e       # NED east velocity [m/s]
float64 vel_d       # NED down velocity [m/s]
float64 vel_acc     # speed accuracy estimate [m/s]
