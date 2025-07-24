# MM-LINS
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/MM_LINS_M3DGR

cd baseline_systems/MM_LINS_M3DGR

catkin_make
```
## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch fast_lio mapping_avia.launch

source devel/setup.bash
roslaunch aloam_velodyne fastlio_ouster64.launch

rosbag play sharp_turn_1.bag /livox/avia/lidar:=/livox/lidar /livox/avia/imu:=/livox/imu
```
