# LIO-SAM
Quickly run on M3DGR, M2DGR-Plus and M2DGR:

## 1.Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/LIO_SAM_M3DGR

cd baseline_systems/LIO_SAM_M3DGR

catkin_make
```

## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch lio_sam run_mid360.launch

rosbag play Dynamic01.bag
```

## 3. Run M2DGR-Plus example

this [link](https://github.com/sjtuyinjie/M2DGR-Benchmark/tree/main/LIO_Sam_M2DGRP)

## 4. Run M2DGR example

this [link](https://github.com/sjtuyinjie/M2DGR-Benchmark/tree/main/LIO_Sam_M2DGRP)



