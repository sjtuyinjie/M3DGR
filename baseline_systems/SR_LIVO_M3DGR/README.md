# sr-livo
##  m2dgr-plus
```
source devel/setup.bash
roslaunch sr_livo livo_m2dgr_plus.launch

# 开启imu坐标转换
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

# 开启lidar格式转换
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play anomaly.bag --topic /rslidar_points /camera/color/image_raw /camera/imu

```

##  m2dgr
```
roslaunch sr_livo livo_m2dgr.launch
rosbag play door_02.bag
```

##  m3dgr
```
source devel/setup.bash

roslaunch livox_repub mid360_repub.launch

roslaunch sr_livo livo_m3dgr_mid360.launch

***
roslaunch sr_livo livo_m3dgr_avia.launch

rosbag play door_02.bag
***
```
