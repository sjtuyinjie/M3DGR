# LINS
Quickly run on M3DGR and M2DGR-Plus:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/LINS_M3DGR

cd baseline_systems/LINS_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
# for mid360
source devel/setup.bash
roslaunch livox_repub mid360_repub.launch

source devel/setup.bash
roslaunch lins run_mid360_m3dgr.launch

# for avia 
source devel/setup.bash
roslaunch livox_repub avia_repub.launch

source devel/setup.bash
roslaunch lins run_avia_m3dgr.launch

rosbag play Dynamic01.bag --clock

```

## 3. Run M2DGR-Plus example
```
source devel/setup.bash
roslaunch lins run_m2dgr_plus.launch

source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

rosbag play tree3.bag --clock

```
