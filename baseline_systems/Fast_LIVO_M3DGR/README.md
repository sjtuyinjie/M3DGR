# fast-livo
##  m2dgr-plus
```
source devel/setup.bash
roslaunch fast_livo mapping_m2dgr_plus.launch

# 开启imu坐标转换
source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

# 开启lidar格式转换
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play tree3.bag --topic /rslidar_points /camera/color/image_raw /camera/imu

```
##  m2dgr
```
source devel/setup.bash
roslaunch fast_livo mapping_m2dgr.launch
rosbag play door_02.bag 
```
##  m3dgr
```
source devel/setup.bash

roslaunch fast_livo mapping_avia_m3dgr.launch

roslaunch fast_livo mapping_mid360_m3dgr.launch

rosbag play door_02.bag 
```
