// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef GRID_MAPPER_H
#define GRID_MAPPER_H

#include <grid_map.h>


class GridMapper{
public:
    GridMapper( GridMap* map, double P_occ, double P_free, double P_prior);
    void updateMap(Vector3d pointWorld, Vector3d PG_T);
    void updateGrid(const Eigen::Vector2d& grid, const double& pmzx);
    double laserInvModel(const double& r, const double& R, const double& cell_size);
    
private:
    GridMap* map_;
    double  P_free_, P_prior_, P_occ_;

}; //class GridMapper

#endif

