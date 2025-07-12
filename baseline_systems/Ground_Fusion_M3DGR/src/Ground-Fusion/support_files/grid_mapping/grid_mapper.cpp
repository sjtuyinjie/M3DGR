// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <grid_mapper.h>

GridMapper::GridMapper ( GridMap* map, double P_occ, double P_free, double P_prior):
        map_(map), P_occ_(P_occ), P_free_(P_free), P_prior_(P_prior)
{

}

void GridMapper::updateMap(Vector3d pointWorld, Vector3d PG_T)
{
    /* 设置遍历的步长，沿着一条激光线遍历 */
    const double& cell_size = map_->getCellSize();
    double inc_step = 1 * cell_size;
    double angle = atan( (pointWorld[1] - PG_T[1]) / (pointWorld[0] - PG_T[0]));
    double z = sqrt((pointWorld[1] - PG_T[1]) * (pointWorld[1] - PG_T[1]) + (pointWorld[0] - PG_T[0]) * (pointWorld[0] - PG_T[0]));
    double cosz = (pointWorld[0] - PG_T[0]) / z;
    double sinz = (pointWorld[1] - PG_T[1]) / z;

    Eigen::Vector2d last_grid(Eigen::Infinity, Eigen::Infinity); //上一步更新的grid位置，防止重复更新

    for(double r = 0; r < z + cell_size; r += inc_step)
    {
        Eigen::Vector2d p_w(r * cosz + PG_T[0], r * sinz + PG_T[1]);

        /* 更新这个grid */
        if(p_w == last_grid) //避免重复更新
            continue;

        updateGrid(p_w, laserInvModel(r, z, cell_size));

        last_grid = p_w;
    }
}

void GridMapper::updateGrid ( const Eigen::Vector2d& grid, const double& pmzx )
{
    /* TODO 这个过程写的太低效了 */
    double log_bel;
    if(  ! map_->getGridLogBel( grid(0), grid(1), log_bel )  ) //获取log的bel
        return;
    log_bel += log( pmzx / (1.0 - pmzx) ); //更新
    map_->setGridLogBel( grid(0), grid(1), log_bel  ); //设置回地图
}

double GridMapper::laserInvModel ( const double& r, const double& R, const double& cell_size )
{
    if(r < ( R - 0.5 * cell_size) )
        return P_free_;
    
    if(r > ( R + 0.5 * cell_size) )
        return P_prior_;
    
    return P_occ_;
}

