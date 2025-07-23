# CT-LIO
Quickly run on M3DGR and M2DGR-Plus:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/CT_LIO_M3DGR

cd CT_LIO_M3DGR

catkin_make -j1
```

## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch ct_lio run_eskf.launch

rosbag play Dynamic01.bag
```

## 3. Run M2DGR-Plus example
```
source devel/setup.bash
rosrun rs_to_velodyne rs_to_velodyne m2dgrplus XYZIRT

source devel/setup.bash
roslaunch imu_transformer ned_to_enu.launch

source devel/setup.bash
roslaunch ct_lio run_eskf.launch

rosbag play tree3.bag
```


