#include "state.h"

state::state()
{
    rotation = Eigen::Quaterniond::Identity();
    translation = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    ba = Eigen::Vector3d::Zero();
    bg = Eigen::Vector3d::Zero();

    fx = 0.0;
    fy = 0.0;
    cx = 0.0;
    cy = 0.0;

    R_imu_camera = Eigen::Matrix3d::Identity();
    t_imu_camera = Eigen::Vector3d::Zero();

    q_world_camera = Eigen::Quaterniond::Identity();
    t_world_camera = Eigen::Vector3d::Zero();

    q_camera_world = Eigen::Quaterniond::Identity();
    t_camera_world = Eigen::Vector3d::Zero();

    fov_margin = 0.005;
    time_td = 0.0;
}

state::state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, 
        const Eigen::Vector3d &velocity_, const Eigen::Vector3d& ba_, const Eigen::Vector3d& bg_)
    : rotation{rotation_}, translation{translation_}, velocity{velocity_}, ba{ba_}, bg{bg_}
{
    fx = 0.0;
    fy = 0.0;
    cx = 0.0;
    cy = 0.0;

    R_imu_camera = Eigen::Matrix3d::Identity();
    t_imu_camera = Eigen::Vector3d::Zero();

    q_world_camera = Eigen::Quaterniond::Identity();
    t_world_camera = Eigen::Vector3d::Zero();

    q_camera_world = Eigen::Quaterniond::Identity();
    t_camera_world = Eigen::Vector3d::Zero();

    fov_margin = 0.005;
    time_td = 0.0;
}

state::state(const state* state_temp)
{
    rotation = state_temp->rotation;
    translation = state_temp->translation;
    velocity = state_temp->velocity;
    ba = state_temp->ba;
    bg = state_temp->bg;

    fx = state_temp->fx;
    fy = state_temp->fy;
    cx = state_temp->cx;
    cy = state_temp->cy;

    R_imu_camera = state_temp->R_imu_camera;
    t_imu_camera = state_temp->t_imu_camera;

    q_world_camera = state_temp->q_world_camera;
    t_world_camera = state_temp->t_world_camera;

    q_camera_world = state_temp->q_camera_world;
    t_camera_world = state_temp->t_camera_world;

    fov_margin = state_temp->fov_margin;
    time_td = state_temp->time_td;
}

void state::release()
{

}