# LIO-Livox (A Robust LiDAR-Inertial Odometry for Livox LiDAR)
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/LIO_Liovx_M3DGR

cd baseline_systems/LIO_Liovx_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch lio_livox mid360.launch

rosbag play Dynamic01.bag
```
