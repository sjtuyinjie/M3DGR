# r2live
##  m2dgr-plus
```
source devel/setup.bash
roslaunch r2live m2dgr_plus.launch

source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play xx.bag

```

##  m3dgr
```
##outdoor
source devel/setup.bash

roslaunch r2live m3dgr_avia.launch

rosbag play xx.bag /livox/avia/lidar:=/livox/lidar

##indoor
roslaunch livox_repub mid360_repub.launch

roslaunch r2live m3dgr_mid360.launch

rosbag play xx.bag

```
