# DM-VIO
Quickly run on M3DGR and M2DGR-Plus:
## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/DM_VIO_M3DGR

cd baseline_systems/DM_VIO_M3DGR

mkdir build && cd build 
cmake ..
make -j4

cd .. && cd dm-vio-ros

export DMVIO_BUILD=../build

catkin_make
```

## 2. Run M3DGR example
```
roscore

source devel/setup.bash
rosrun dmvio_ros node calib=./src/configs/m3dgr.txt imuCalib=./src/configs/m3dgr_imu.yaml settingsFile=./src/configs/m3dgr_setnoise.yaml mode=1 nogui=0 preset=1 useimu=1 quiet=1 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1

rosrun image_transport republish compressed in:=/camera/color/image_raw  raw  out:=/camera/color/image_raw

rosbag play dynamic_person_1.bag 

```

## 3. Run M2DGR-Plus example
```
roscore

source devel/setup.bash
rosrun dmvio_ros node calib=./src/configs/m2dgrp.txt imuCalib=./src/configs/m2dgr_plus_imu.yaml settingsFile=./src/configs/m2dgr_plus_setnoise.yaml mode=1 nogui=0 preset=1 useimu=1 quiet=1 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1

rosbag play tree3.bag
```
