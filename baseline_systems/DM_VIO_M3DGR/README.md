# DM-VIO
## m3dgr
```
mkdir build && cd build 
cmake ..
make -j4

cd .. && cd dm-vio-ros

export DMVIO_BUILD=/media/zjj/Elements/CQU_ZJJ/M3DGR/DM_VIO_M3DGR/build

catkin_make

source devel/setup.bash

rosrun dmvio_ros node calib=./src/configs/m3dgr.txt imuCalib=./src/configs/m3dgr_imu.yaml settingsFile=./src/configs/m3dgr_setnoise.yaml mode=1 nogui=0 preset=1 useimu=1 quiet=1 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1

roscore

rosrun image_transport republish compressed in:=/camera/color/image_raw  raw  out:=/camera/color/image_raw

rosbag play dynamic_person_1.bag 

# /camera/color/image_raw:=/cam0/image_raw /camera/imu:=/imu0
```

## m2dgr_plus
```
source devel/setup.bash

roscore

rosrun dmvio_ros node calib=./src/configs/m2dgrp.txt imuCalib=./src/configs/m2dgr_plus_imu.yaml settingsFile=./src/configs/m2dgr_plus_setnoise.yaml mode=1 nogui=0 preset=1 useimu=1 quiet=1 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1

rosbag play tree3.bag /camera/color/image_raw:=/cam0/image_raw /camera/imu:=/imu0
```
