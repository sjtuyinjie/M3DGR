// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <grid_map.h>

GridMap::GridMap ( const int& size_x, const int& size_y, const int& init_x, const int& init_y, const double& cell_size ) :
    size_x_ ( size_x ), size_y_ ( size_y ), init_x_ ( init_x ), init_y_ ( init_y ), cell_size_ ( cell_size )
{
    bel_data_.resize ( size_x_, size_y_ );
    bel_data_.setOnes() *= 0.5; //全部设为0.5的概率
    
    /* 为opencv图片显示相关 */
    m_one_.resize(size_x_, size_y_);
    m_one_.setOnes();
    m_show_.resize(size_x_, size_y_);
    m_show_.setOnes() * 0.5;
}

bool GridMap::getIdx ( const double& x, const double& y, Eigen::Vector2i& idx )
{
    int xidx = cvFloor( x / cell_size_ ) + init_x_;
    int yidx  = cvFloor( y /cell_size_ )+ init_y_;
    
   
    if((xidx < 0) || (yidx < 0) || (xidx >= size_x_) || (yidx >= size_y_)){
        cout << xidx << " " << yidx << endl;
        return false;
    }

    idx << xidx , yidx;
    return true;
}

bool GridMap::getGridBel ( const double& x, const double& y, double& bel)
{
    Eigen::Vector2i idx;
    if(!getIdx(x, y, idx))
        return false;
    bel = bel_data_(idx(0), idx(1));
    return true;
}

bool GridMap::setGridBel ( const double& x, const double& y, const double& bel )
{
    Eigen::Vector2i idx;
    if(!getIdx(x, y, idx))
        return false;
    bel_data_(idx(0), idx(1)) = bel;
    return true;
}

bool GridMap::getGridLogBel ( const double& x, const double& y, double& log_bel )
{
    double bel;
    if(!getGridBel(x, y, bel))
        return false;
    log_bel = log( bel / (1.0-bel));
    return true;
}

bool GridMap::setGridLogBel ( const double& x, const double& y, const double& log_bel )
{
    double bel = 1.0 - 1.0 / (1 + exp(log_bel));
    if( !setGridBel(x, y, bel) )
        return false;
    return true;
}

double GridMap::getCellSize()
{
    return cell_size_;
}

cv::Mat GridMap::toCvMat()
{
   /* 构造opencv mat */
    m_show_ = m_one_ - bel_data_; //翻转数值
    /* 构造出opencv格式显示 */
    cv::Mat map;
    eigen2cv(m_show_, map);
    //cv::Mat map(cv::Size(size_x_, size_y_), CV_64FC1, m_show_.data(), cv::Mat::AUTO_STEP);

    /* 翻转 */
   cv::flip(map, map, 0);
   
   return map;
}

/*
void GridMap::toRosOccGridMap( const std::string& frame_id, nav_msgs::OccupancyGrid& occ_grid)
{
    occ_grid.header.frame_id = frame_id;
    occ_grid.header.stamp = ros::Time::now();
    
    occ_grid.info.width = size_x_;
    occ_grid.info.height = size_y_;
    occ_grid.info.resolution = cell_size_;
     occ_grid.info.origin.position.x = -init_x_*cell_size_;
     occ_grid.info.origin.position.y = -init_y_*cell_size_;
    
     const int N = size_x_ * size_y_;
    for(size_t i= 0; i < N; i ++)
    {
        double& value = bel_data_.data()[i];
        if(value == 0.5)
            occ_grid.data.push_back( -1);
        else
            occ_grid.data.push_back( value * 100);
    }
}
*/

void GridMap::saveMap ( const std::string& img_dir, const std::string& cfg_dir )
{
    /* 保存图片 */
    cv::Mat img = toCvMat();
    img = img * 255;
    cv::imwrite(img_dir, img);
    
    /* 保存配置 */
    std::ofstream  file;
    file.open(cfg_dir); 
    file << "map:"<< std::endl
    << "  size_x: " << size_x_ << std::endl
    << "  size_y: " << size_y_ << std::endl
    << "  init_x: " << init_x_ << std::endl
    << "  init_y: " << init_y_ << std::endl
    << "  cell_size: " << cell_size_ << std::endl;
}








