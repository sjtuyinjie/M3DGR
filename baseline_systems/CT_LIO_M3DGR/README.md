# CT-LIO
Quickly run on M3DGR, M2DGR-Plus and M2DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/CT_LIO_M3DGR

cd baseline_systems/CT_LIO_M3DGR

catkin_make -j1
```

## 2. Run M3DGR example
```
cp -f ./src/ct_lio/config/mapping_m3dgr.yaml ./src/ct_lio/config/mapping.yaml

source devel/setup.bash
roslaunch ct_lio run_eskf.launch

rosbag play Dynamic01.bag
```

## 3. Run M2DGR-Plus example
```
cp -f ./src/ct_lio/config/mapping_m2dgrp.yaml ./src/ct_lio/config/mapping.yaml

source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

source devel/setup.bash
roslaunch ct_lio run_eskf.launch

rosbag play tree3.bag
```
## 2. Run M2DGR example
```
cp -f ./src/ct_lio/config/mapping_m2dgr.yaml ./src/ct_lio/config/mapping.yaml

source devel/setup.bash
roslaunch ct_lio run_eskf.launch

rosbag play door_02.bag
```

