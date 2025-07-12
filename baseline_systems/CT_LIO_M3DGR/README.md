# ct-lio

catkin_make -j1

## m3dgr
```
source devel/setup.bash

roslaunch ct_lio run_eskf.launch
```

# m2dgrp
```
source devel/setup.bash

rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosrun rs_to_velodyne rs_to_velodyne XYZI XYZIRT

rosrun rs_to_velodyne rs_to_velodyne XYZIRT XYZIRT

source devel/setup.bash

roslaunch imu_transformer ned_to_enu.launch

source devel/setup.bash

roslaunch ct_lio run_eskf.launch

```


