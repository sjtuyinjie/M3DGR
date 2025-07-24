# R3LIVE
Quickly run on M3DGR, M2DGR-Plus and M2DGR:

# 1. Dependency and compilation
```
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install

sudo apt-get install libcgal-dev pcl-tools 
sudo apt-get install ros-noetic-tf2-sensor-msgs

git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/R3LIVE_M3DGR

cd R3LIVE_M3DGR

catkin_make
```
## 2. Run M3DGR example
```
# outdoor
source devel/setup.bash
roslaunch r3live r3live_m3dgr_avia.launch

rosbag play Dynamic03.bag /livox/avia/lidar:=/livox/lidar

# indoor
source devel/setup.bash
roslaunch r3live r3live_m3dgr_mid360.launch

rosbag play Dynamic01.bag /livox/mid360/lidar:=/livox/lidar
```

And you can use  Realsense D435i IMU:

```
# outdoor
source devel/setup.bash
roslaunch r3live r3live_m3dgr_avia_cimu.launch

source devel/setup.bash
roslaunch imu_transformer m3dgr_avia.launch

rosbag play Dynamic03.bag /livox/avia/lidar:=/livox/lidar

# indoor
source devel/setup.bash
roslaunch r3live r3live_m3dgr_mid360_cimu.launch

source devel/setup.bash
roslaunch imu_transformer m3dgr_mid360.launch

rosbag play Dynanmic01.bag /livox/mid360/lidar:=/livox/lidar
```


##  3. Run M2DGR-Plus example

```
source devel/setup.bash
roslaunch r3live r3live_m2dgr_plus_v.launch

source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play tree3.bag --topic /rslidar_points /camera/color/image_raw /camera/imu
```

## 4. Run M2DGR example
```
source devel/setup.bash
roslaunch r3live r3live_m2dgr.launch

rosbag play door_02.bag
```

