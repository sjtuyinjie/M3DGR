# Fast-LIO2
Quickly run on M3DGR and M2DGR-Plus:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/FAST_LIO2_M3DGR

cd FAST_LIO2_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
# for avia
source devel/setup.bash
roslaunch fast_lio mapping_avia.launch

# for mid360
source devel/setup.bash   
roslaunch fast_lio mapping_mid360.launch
```

## 3. Run N2DGR-Plus example
```
source devel/setup.bash
roslaunch fast_lio mapping_m2dgrp.launch

source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch
```
