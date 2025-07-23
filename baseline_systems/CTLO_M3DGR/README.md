# CT-LIO
Quickly run on M3DGR:

## 1. Compile
```
git clone https://github.com/sjtuyinjie/M3DGR.git && cd M3DGR && git sparse-checkout set --no-cone baseline_systems/VINS_Mono_M3DGR

cd VINS_Mono_M3DGR

catkin_make
```
## 2. Run M3DGR example
```
source devel/setup.bash
roslaunch ctlo run.launch
```
