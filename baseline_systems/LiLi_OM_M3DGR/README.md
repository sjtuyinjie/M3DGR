# LiLi-OM (LIvox LiDAR-Inertial Odometry and Mapping)
Quickly run on M3DGR :

## 1. Complie
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/LiLi_OM_M3DGR

cd LiLi_OM_M3DGR

cd ws_livox && mkdir build && cd build
camke .. && make

cd ../../LiLi-OM && mkdir build && cd build

source ../../ws_livox/devel/setup.bash

camke .. && make

source devel/setup.bash

chmod +x ../scripts/InternalImuUnitConverter.py

cd ../../LiLi-OM-ROT && mkdir build && cd build
cmake .. && make

cd ../../livox_repub && mkdir build && cd build
cmake .. && make
```

## 2. Run M3DGR example
```
## LiLi-OM:
cd LiLi-OM/build

source devel/setup.bash

roslaunch lili_om run_m3dgr_avia.launch

rosbag play Dynamic01.bag


## LiLi-OM-ROT:
cd livox_repub/build

source devel/setup.bash

roslaunch livox_repub mid360_repub.launch

cd LiLi-OM-ROT/build

source devel/setup.bash

roslaunch lili_om_rot run_m3dgr_mid360.launch

rosbag play Dynamic01.bag
```




