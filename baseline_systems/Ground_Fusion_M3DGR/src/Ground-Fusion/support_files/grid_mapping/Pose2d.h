// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef POSE2D_H
#define POSE2D_H

#include <eigen3/Eigen/Core>
#define PI 3.1415926

class Pose2d{
public:
    Pose2d(){
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
    }
    Pose2d(double x, double y, double theta):x_(x), y_(y), theta_(theta){}
    
    const Pose2d operator*(const Pose2d& p2)   
    {  
        Pose2d p;
        Eigen::Matrix2d R;
        R << cos(theta_), -sin(theta_),
        sin(theta_), cos(theta_);
        Eigen::Vector2d pt2(p2.x_, p2.y_);          
        Eigen::Vector2d pt = R * pt2 + Eigen::Vector2d(x_, y_);
        
        p.x_ = pt(0);
        p.y_ = pt(1);
        p.theta_ = theta_ + p2.theta_;
        NormAngle( p.theta_);
        return p;
    }  
    
    const Eigen::Vector2d operator*(const Eigen::Vector2d& p)   
    {  
        Eigen::Matrix2d R;
        R << cos(theta_), -sin(theta_),
        sin(theta_), cos(theta_);
        Eigen::Vector2d t(x_, y_);
        return R * p + t;
    }  
    
    
    Pose2d inv()
    {
        double x = -( cos(theta_) * x_ + sin(theta_) * y_);
        double y = -( -sin(theta_) * x_ + cos(theta_) * y_);
        double theta = -theta_;
        return Pose2d(x, y, theta);
    }
    
    void NormAngle ( double& angle )
    {        if( angle >= PI)
            angle -= 2.0*PI;
        if( angle < -PI)
            angle += 2.0*PI;
    }
    double x_, y_, theta_; 
}; //class Pose2d


#endif

