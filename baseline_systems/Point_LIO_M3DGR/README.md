# Point-LIO

## m2dgrp
```
source devel/setup.bash

roslaunch point_lio mapping_m2dgrp.launch

source devel/setup.bash

rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

source devel/setup.bash

roslaunch imu_transformer ned_to_enu.launch
```

## m3dgr-limu
```
//avia

source devel/setup.bash

roslaunch point_lio mapping_avia.launch 

//mid360

source devel/setup.bash

roslaunch point_lio mapping_mid360.launch 
```
## m3dgr-cimu
```
//avia

source devel/setup.bash

roslaunch point_lio mapping_m3dgr_avia.launch 

//mid360

source devel/setup.bash

roslaunch point_lio mapping_m3dgr_mid360.launch 
```
### evo
```
evo_ape tum gtlight_dark1.txt light_dark1.txt -p -va  --t_max_diff 0.041
```

