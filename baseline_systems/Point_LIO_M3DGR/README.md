# Point-LIO
Quickly run on M3DGR and M2DGR-Plus:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/Point_LIO_M3DGR

cd baseline_systems/Point_LIO_M3DGR

catkin_make
```

## 2. Run M3DGR example 
```
# avia
source devel/setup.bash
roslaunch point_lio mapping_avia.launch
(roslaunch point_lio mapping_m3dgr_avia.launch # for camera/imu)

# mid360
source devel/setup.bash
roslaunch point_lio mapping_mid360.launch
(roslaunch point_lio mapping_m3dgr_mid360.launch # for camera/imu)

rosbag play Dynamic01.bag
```

## 3. Run M2DGR-Plus example
```
source devel/setup.bash
roslaunch point_lio mapping_m2dgrp.launch

source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

rosbag play tree3.bag
```

