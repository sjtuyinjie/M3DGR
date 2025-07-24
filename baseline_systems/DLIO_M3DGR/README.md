# Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/DLIO_M3DGR

cd baseline_systems/DLIO_M3DGR

catkin_make
```
## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch direct_lidar_inertial_odometry dlio_m3dgr_mid360.launch

rosbag play Dynamic01.bag
```
