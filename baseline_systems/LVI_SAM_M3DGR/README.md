# lvi_sam
## M2DGR
```
source devel/setup.bash

roslaunch lvi_sam m2dgr_run.launch

```
## M2DGR-Plus
```
source devel/setup.bash

roslaunch lvi_sam m2dgr_plus_run.launch

```
## M3DGR
```
//outdoor
source devel/setup.bash

roslaunch lvi_sam m3dgr_run.launch

source devel/setup.bash

roslaunch livox_repub avia_repub.launch


//indoor
source devel/setup.bash

roslaunch lvi_sam 360m3dgr_run.launch

source devel/setup.bash

roslaunch livox_repub mid360_repub.launch

```
