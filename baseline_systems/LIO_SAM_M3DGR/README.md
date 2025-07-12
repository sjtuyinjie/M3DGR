# lio-sam
## M2DGR
```
source devel/setup.bash

roslaunch lio_sam_6axis m2dgr_run.launch
```
## M2DGR-PLUS

```
source devel/setup.bash

roslaunch lio_sam m2dgrplus_run.launch
```
## M3DGR
```
//outdoor:
source devel/setup.bash

roslaunch lio_sam_6axis avia_m3dgr_run.launch

roslaunch lio_sam_6axis cavia_m3dgr_run.launch

source devel/setup.bash

roslaunch livox_repub avia_repub.launch

//indoor:
roslaunch lio_sam_6axis mid360_m3dgr_run.launch

roslaunch lio_sam_6axis cmid360_m3dgr_run.launch

roslaunch livox_repub mid360_repub.launch
```
