//
// Created by hk on 8/24/23.
//


#ifndef FAST_LIO_CLOUD_PROCESS_HPP
#define FAST_LIO_CLOUD_PROCESS_HPP
#include <common_lib.h>
#include <ikd-Tree/ikd_Tree.h>
#include <voxel_map_util.hpp>

extern float mid2min, planarity, angle_threshold;
extern int num_surfel, surfel_points_min, surfel_points_max;
extern bool check_normal;
//const int large_scale_flag = -2;
bool cloudSurfel(const vector<VoxelInfo*> & voxels, Matrix<float, 4, 1> &pca_result, V3F& centroid, const float& threshold)
{
    float s_xx= 0.0, s_xy = 0.0, s_xz = 0.0, s_yy = 0.0, s_yz = 0.0, s_zz = 0.0;
    float mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
    float num_points = 0.0;
//    centroid = Eigen::Vector3f (0, 0, 0);
    Eigen::Vector3f mean_normal(0, 0, 0);
    vector<Eigen::Vector3f> valid_mean_xyz(voxels.size());
    int num_valid = 0;
    for (int i = 0; i < (int) voxels.size(); ++i) {
        const auto & v = voxels[i];
        if (!v)
            continue;
//        lock_guard<mutex> lock(v->updating_mutex_lock);
        if (v->num_points >= 0 && v->num_points < surfel_points_min) // not enough points
            continue;
        float n = (float)abs(v->num_points);
        num_points += n;
//        const PointType & p = cloud[i];
        s_xx += v->s_xx; s_xy += v->s_xy; s_xz += v->s_xz;
        s_yy += v->s_yy; s_yz += v->s_yz; s_zz += v->s_zz;

        mean_x += n * v->mean_x;
        mean_y += n * v->mean_y;
        mean_z += n * v->mean_z;

        mean_normal += n * v->normal;
        valid_mean_xyz[num_valid](0) = v->mean_x;
        valid_mean_xyz[num_valid](1) = v->mean_y;
        valid_mean_xyz[num_valid](2) = v->mean_z;
        ++num_valid;
    }

    // todo
    if (num_points < surfel_points_min * NUM_MATCH_POINTS * 0.5)
        return false;

    mean_x /= num_points;
    mean_y /= num_points;
    mean_z /= num_points;
    centroid(0) = mean_x;
    centroid(1) = mean_y;
    centroid(2) = mean_z;
    mean_normal = mean_normal / num_points;
    mean_normal.normalize();
//    valid_mean_xyz.resize(num_valid);
//    ROS_INFO("mean xyz %f %f %f", mean_x, mean_y, mean_z);

    Eigen::Matrix3f cov;
    cov(0, 0) = s_xx / num_points - mean_x * mean_x;
    cov(1, 1) = s_yy / num_points - mean_y * mean_y;
    cov(2, 2) = s_zz / num_points - mean_z * mean_z;

    cov(0, 1) = s_xy / num_points - mean_x * mean_y;
    cov(1, 0) = cov(0, 1);

    cov(0, 2) = s_xz / num_points - mean_x * mean_z;
    cov(2, 0) = cov(0, 2);

    cov(1, 2) = s_yz / num_points - mean_y * mean_z;
    cov(2, 1) = cov(1, 2);

    // Compute the product cloud_demean * cloud_demean^T
//    Eigen::Matrix3f alpha = static_cast<Eigen::Matrix3f> (cloud_demean.topRows<3> () * cloud_demean.topRows<3> ().transpose ());

    Eigen::Matrix3f eigenvectors_;
    Eigen::Vector3f eigenvalues_;

    // Compute eigen vectors and values
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd (cov);
    // Organize eigenvectors and eigenvalues in ascendent order
    // lambda 1 > lambda 2 > lambda 3
    for (int i = 0; i < 3; ++i)
    {
        eigenvalues_[i] = evd.eigenvalues () [2-i];
        eigenvectors_.col (i) = evd.eigenvectors ().col (2-i);
    }

//    float cos = eigenvectors_.col (2).dot(voxels[0]->normal);
    if (eigenvalues_[0] == eigenvalues_[2] || eigenvalues_[0] == 0.0)
        return false;
    float pho = 2.0 * (eigenvalues_[1] - eigenvalues_[2]) / (eigenvalues_[0] + eigenvalues_[1] + eigenvalues_[2]);
    float mid_min = eigenvalues_[1] / eigenvalues_[2];

    // n * p + d = 0;
    Eigen::Vector3f normal = eigenvectors_.col (2);
    if (mean_normal.dot(normal) < 0)
        normal *= -1;
    pca_result.head(3) = normal;
    pca_result(3) = -centroid.dot(normal); // d

    for (int j = 0; j < num_valid; j++)
    {
//        while (voxels[j]->flag_updating) /// !!!caution
//            usleep(2);
//        pthread_mutex_lock(&voxels[j]->updating_mutex_lock);

        if (fabs(normal.dot(valid_mean_xyz[j]) + pca_result(3)) > threshold)
        {
            return false;
        }
//        pthread_mutex_lock(&voxels[j]->updating_mutex_lock);
    }
    return true;
}

///code from VoxelMap
void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov) {
    // if z=0, error will occur in calcBodyCov. To be solved
    float z_tmp = pb[2];
    if (z_tmp == 0) {
        z_tmp = 0.001;
    }
    float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + z_tmp * z_tmp);
//    float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
    float range_var = range_inc * range_inc;
    Eigen::Matrix2d direction_var;
    // (angle_cov^2, 0,
    //  0, angle_cov^2)
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
    Eigen::Vector3d direction(pb);
    direction.normalize();
    Eigen::Matrix3d direction_hat; // w^
    direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
    //direction dot base_vector1 = 0
    Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2)); //(1, 1, -(x+y)/z), not unique
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N; //N = [base_vector1, base_vector2]
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
            base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N; // (d * w^ * N )in the paper
    //cov = w * var_d * w^T + A * var_w * A^T
    cov = direction * range_var * direction.transpose() +
          A * direction_var * A.transpose();
}

void calcBodyCov(PointType &pb, const float range_cov, const float degree_cov, Eigen::Matrix3f &cov) {
    // if z=0, error will occur in calcBodyCov. To be solved
    float z_tmp = pb.z;
    if (z_tmp == 0) {
        z_tmp = 0.001;
    }
    float range = sqrt(pb.x* pb.x + pb.y * pb.y + z_tmp * z_tmp);
//    float range = sqrt(pb.x* pb.x + pb.y * pb.y + pb.z * pb.z);
    float range_var = range_cov * range_cov;
    Eigen::Matrix2f direction_var;
    // (angle_cov^2, 0,
    //  0, angle_cov^2)
    direction_var << pow(sin(DEG2RAD(degree_cov)), 2), 0, 0, pow(sin(DEG2RAD(degree_cov)), 2);
    Eigen::Vector3f direction = pb.getVector3fMap();
    direction.normalize();
    Eigen::Matrix3f direction_hat; // w^
    direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
    //direction dot base_vector1 = 0
    Eigen::Vector3f base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2)); //(1, 1, -(x+y)/z), not unique
    base_vector1.normalize();
    Eigen::Vector3f base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<float, 3, 2> N; //N = [base_vector1, base_vector2]
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
            base_vector1(2), base_vector2(2);
    Eigen::Matrix<float, 3, 2> A = range * direction_hat * N; // (d * w^ * N )in the paper
    //cov = w * var_d * w^T + A * var_w * A^T
    cov = direction * range_var * direction.transpose() +
          A * direction_var * A.transpose();

//    pb.recordPointCovFromMatrix(cov);
};

void GetUpdatePlane(const OctoTree *current_octo, const int pub_max_voxel_layer,
                    vector<Plane> &plane_list) {
    if (current_octo->layer_ > pub_max_voxel_layer) {
        return;
    }
    if (current_octo->plane_ptr_->is_update) {
        plane_list.push_back(*current_octo->plane_ptr_);
    }
    if (current_octo->layer_ < current_octo->max_layer_) {
        if (!current_octo->plane_ptr_->is_plane) {
            for (size_t i = 0; i < 8; i++) {
                if (current_octo->leaves_[i] != nullptr) {
                    GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer,
                                   plane_list);
                }
            }
        }
    }
    return;
}

void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g,
            uint8_t &b) {
    r = 255;
    g = 255;
    b = 255;

    if (v < vmin) {
        v = vmin;
    }

    if (v > vmax) {
        v = vmax;
    }

    double dr, dg, db;

    if (v < 0.1242) {
        db = 0.504 + ((1. - 0.504) / 0.1242) * v;
        dg = dr = 0.;
    } else if (v < 0.3747) {
        db = 1.;
        dr = 0.;
        dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
    } else if (v < 0.6253) {
        db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
        dg = 1.;
        dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
    } else if (v < 0.8758) {
        db = 0.;
        dr = 1.;
        dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
    } else {
        db = 0.;
        dg = 0.;
        dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
    }

    r = (uint8_t) (255 * dr);
    g = (uint8_t) (255 * dg);
    b = (uint8_t) (255 * db);
}

void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec,
                     const Eigen::Vector3d &z_vec,
                     geometry_msgs::Quaternion &q) {

    Eigen::Matrix3d rot;
    rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0),
            z_vec(1), z_vec(2);
    Eigen::Matrix3d rotation = rot.transpose();
    Eigen::Quaterniond eq(rotation);
    q.w = eq.w();
    q.x = eq.x();
    q.y = eq.y();
    q.z = eq.z();
}

void CalcQuation(const Eigen::Vector3d &vec, const int axis,
                 geometry_msgs::Quaternion &q) {
    Eigen::Vector3d x_body = vec;
    Eigen::Vector3d y_body(1, 1, 0);
    if (x_body(2) != 0) {
        y_body(2) = -(y_body(0) * x_body(0) + y_body(1) * x_body(1)) / x_body(2);
    } else {
        if (x_body(1) != 0) {
            y_body(1) = -(y_body(0) * x_body(0)) / x_body(1);
        } else {
            y_body(0) = 0;
        }
    }
    y_body.normalize();
    Eigen::Vector3d z_body = x_body.cross(y_body);
    Eigen::Matrix3d rot;

    rot << x_body(0), x_body(1), x_body(2), y_body(0), y_body(1), y_body(2),
            z_body(0), z_body(1), z_body(2);
    Eigen::Matrix3d rotation = rot.transpose();
    if (axis == 2) {
        Eigen::Matrix3d rot_inc;
        rot_inc << 0, 0, 1, 0, 1, 0, -1, 0, 0;
        rotation = rotation * rot_inc;
    }
    Eigen::Quaterniond eq(rotation);
    q.w = eq.w();
    q.x = eq.x();
    q.y = eq.y();
    q.z = eq.z();
}

M3D calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc) {
    float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
    float range_var = range_inc * range_inc;
    Eigen::Matrix2d direction_var;
    // (angle_cov^2, 0,
    //  0, angle_cov^2)
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
    Eigen::Vector3d direction(pb);
    direction.normalize();
    Eigen::Matrix3d direction_hat; // w^
    direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
    //direction dot base_vector1 = 0
    Eigen::Vector3d base_vector1(1, 1,
                                 -(direction(0) + direction(1)) / direction(2)); //(1, 1, -(x+y)/z), not unique
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N; //N = [base_vector1, base_vector2]
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
            base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N; // (d * w^ * N )in the paper
    //cov = w * var_d * w^T + A * var_w * A^T
    return direction * range_var * direction.transpose() +
           A * direction_var * A.transpose();
};FAST_LIO_CLOUD_PROCESS_HPP

bool mergeVoxelCheck(const vector<VoxelInfo*>& voxels, const Eigen::Vector3f& body2point, const PointType & point_world,
                 Matrix<float, 4, 1> &pca_plane, const float& threshold)
{
    float sum_diff_angle = 0; // angle between normals
    int num_fail_normal = 0;

    float s_xx= 0.0, s_xy = 0.0, s_xz = 0.0, s_yy = 0.0, s_yz = 0.0, s_zz = 0.0;
    float mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
    float num_points = 0.0;
    int num_valid_voxels = 0;
    Eigen::Vector3f mean_normal(0, 0, 0);
    Eigen::Vector3f centroid(0, 0, 0);
    vector<Eigen::Vector3f> valid_mean_xyz(voxels.size());
    for (int j = 0; j < voxels.size(); j++) {
        //check map points normal direction, to sure lidar can see it
        if (!voxels[j])
            continue;

        VoxelInfo* v = voxels[j];

        // check normal consistency
        const Eigen::Vector3f& voxel_normal = v->normal;
        if (check_normal && voxel_normal.dot(body2point) > 0) {
            ++num_fail_normal;
            continue;
        }
        sum_diff_angle += acos(point_world.getNormalVector3fMap().dot(voxel_normal));

        // check enough points in voxel
        if (v->num_points >= 0 && v->num_points < surfel_points_min) // not enough points
            continue;

        float n = (float)abs(v->num_points);
        num_points += n;
//        const PointType & p = cloud[i];
        s_xx += v->s_xx; s_xy += v->s_xy; s_xz += v->s_xz;
        s_yy += v->s_yy; s_yz += v->s_yz; s_zz += v->s_zz;

        mean_x += n * v->mean_x;
        mean_y += n * v->mean_y;
        mean_z += n * v->mean_z;

        mean_normal += n * v->normal;

        // copy valid mean position
        valid_mean_xyz[num_valid_voxels](0) = v->mean_x;
        valid_mean_xyz[num_valid_voxels](1) = v->mean_y;
        valid_mean_xyz[num_valid_voxels](2) = v->mean_z;
        ++num_valid_voxels;
    }

    mean_x /= num_points;
    mean_y /= num_points;
    mean_z /= num_points;
    centroid(0) = mean_x;
    centroid(1) = mean_y;
    centroid(2) = mean_z;
    mean_normal = mean_normal / num_points;
    mean_normal.normalize();

    Eigen::Matrix3f cov;
    cov(0, 0) = s_xx - num_points * mean_x * mean_x;
    cov(1, 1) = s_yy - num_points * mean_y * mean_y;
    cov(2, 2) = s_zz - num_points * mean_z * mean_z;

    cov(0, 1) = s_xy - num_points * mean_x * mean_y;
    cov(1, 0) = cov(0, 1);

    cov(0, 2) = s_xz - num_points * mean_x * mean_z;
    cov(2, 0) = cov(0, 2);

    cov(1, 2) = s_yz - num_points * mean_y * mean_z;
    cov(2, 1) = cov(1, 2);

    Eigen::Matrix3f eigenvectors_;
    Eigen::Vector3f eigenvalues_;

    // Compute eigen vectors and values
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> evd (cov);
    // Organize eigenvectors and eigenvalues in ascendent order
    // lambda 1 > lambda 2 > lambda 3
    for (int i = 0; i < 3; ++i)
    {
        eigenvalues_[i] = evd.eigenvalues () [2-i];
        eigenvectors_.col (i) = evd.eigenvectors ().col (2-i);
    }

    if (eigenvalues_[0] == eigenvalues_[2] || eigenvalues_[0] == 0.0)
        return false;
    float pho = 2.0 * (eigenvalues_[1] - eigenvalues_[2]) / (eigenvalues_[0] + eigenvalues_[1] + eigenvalues_[2]);
    float mid_min = eigenvalues_[1] / eigenvalues_[2];

    // n * p + d = 0;
    Eigen::Vector3f normal = eigenvectors_.col (2);
    if (mean_normal.dot(normal) < 0)
        normal *= -1;
    pca_plane.head(3) = normal;
    pca_plane(3) = -centroid.dot(normal); // d

    for (const auto & valid_mean : valid_mean_xyz)
        if (fabs(pca_plane.head(3).dot(valid_mean) + pca_plane(3)) > threshold)
            return false;

    V3F cent2point(point_world.getVector3fMap() - centroid);
    pca_plane(3) = normal.dot(cent2point); //pca_plane(3) record point to surfel distance

    return true;
}

bool singleVoxelCheck(const vector<VoxelInfo*>& voxels, const Eigen::Vector3f& body2point, const PointType & point_world,
                 Matrix<float, 4, 1> &pca_result, const float& threshold)
{
    // idea from voxelMap
    double prob = 0.0;
    float dist_min = 999.99;
    for (const VoxelInfo* vox : voxels) {
        if (!vox)
            continue;
        if (vox->num_points > 0 && vox->num_points < surfel_points_min)
            continue;
        if (check_normal && vox->normal.dot(body2point) > 0)
            continue;

        // todo 3 * standard deviation
//                while (voxel_info->flag_updating) /// !!!caution
//                    usleep(2);
//                lock_guard<mutex> lock(voxel_info->updating_mutex_lock);

        V3F plane_centroid(vox->mean_x, vox->mean_y, vox->mean_z);
        V3F plane_normal(vox->normal);

        V3F cent2point(point_world.getVector3fMap() - plane_centroid);
        float p2surfel = plane_normal.dot(cent2point);
        float abs_dist = fabs(p2surfel);
        //todo 从不确定度判断最佳surfel
        if (abs_dist < threshold && abs_dist < dist_min) {
            dist_min = abs_dist;
            pca_result.head(3) = plane_normal;
            pca_result(3) = p2surfel;  //pca_plane(3) record point to surfel distance
        }
    }
    if (dist_min < threshold)
        return true;
    return false;
}
#endif //FAST_LIO_CLOUD_PROCESS_HPP
