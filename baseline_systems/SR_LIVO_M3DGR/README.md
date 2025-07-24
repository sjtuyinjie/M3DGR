# SR-LIVO
Quickly run on M3DGR, M2DGR-Plus and M2DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/SR_LIVO_M3DGR

cd SR_LIVO_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch livox_repub mid360_repub.launch

source devel/setup.bash
roslaunch sr_livo livo_m3dgr_avia.launch

rosbag play Dynamic01.bag
```

## 3. Run M2DGR-Plus example
```
source devel/setup.bash
roslaunch sr_livo livo_m2dgr_plus.launch

source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play anomaly.bag --topic /rslidar_points /camera/color/image_raw /camera/imu
```

## 4. Run M2DGR example
```
source devel/setup.bash
roslaunch sr_livo livo_m2dgr.launch

rosbag play door_02.bag
```
