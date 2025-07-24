# R2live
Quickly run on M3DGR and M2DGR-Plus:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/R2LIVE_M3DGR

cd R2LIVE_M3DGR

catkin_make
```

##  2. Run M3DGR example
```
# outdoor
source devel/setup.bash
roslaunch r2live m3dgr_avia.launch

rosbag play Dynamic03.bag /livox/avia/lidar:=/livox/lidar

# indoor
source devel/setup.bash
roslaunch livox_repub mid360_repub.launch

source devel/setup.bash
roslaunch r2live m3dgr_mid360.launch

rosbag play Dynamic01.bag
```

##  3. Run M2DGR-Plus example
```
source devel/setup.bash
roslaunch r2live m2dgr_plus.launch

source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play tree3.bag

```
