# LiLi-OM (LIvox LiDAR-Inertial Odometry and Mapping)
## complie
```
cd LiLi-OM/build

source ../../ws_livox/devel/setup.bash

source devel/setup.bash

chmod +x ../scripts/InternalImuUnitConverter.py

cd LiLi-OM-ROT/build
```
## m3dgr
```
## LiLi-OM:
cd LiLi-OM/build

source devel/setup.bash

roslaunch lili_om run_m3dgr_avia.launch

rosbag play 


## LiLi-OM-ROT:
cd livox_repub/build

source devel/setup.bash

roslaunch livox_repub mid360_repub.launch

cd LiLi-OM-ROT/build

source devel/setup.bash

roslaunch lili_om_rot run_m3dgr_mid360.launch

rosbag play 
```
frame_id:
cloud: lili_om_rot




