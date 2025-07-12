# Fast-LIO
## M3DGR
```
source devel/setup.bash

roslaunch fast_lio mapping_avia.launch

roslaunch fast_lio mapping_mid360.launch
```
## m2dgrp
```
source devel/setup.bash

roslaunch fast_lio mapping_m2dgrp.launch

source devel/setup.bash

rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

source devel/setup.bash

roslaunch imu_transformer ned_to_enu.launch
```
