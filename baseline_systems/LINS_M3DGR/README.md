# LINS
## m2dgr_plus
```
source devel/setup.bash
roslaunch lins run_m2dgr_plus.launch

source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play xx.bag --clock

```
## m3dgr
```
source devel/setup.bash

roslaunch livox_repub mid360_repub.launch

roslaunch lins run_mid360_m3dgr.launch
 

source devel/setup.bash

roslaunch livox_repub avia_repub.launch

roslaunch lins run_avia_m3dgr.launch

rosbag play xx.bag --clock

```
