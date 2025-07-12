#include "ikd_Tree.h"
//#include <pcl/common/pca.h>

//#include "voxel_map_util.hpp"
/*
Description: ikd-Tree: an incremental k-d tree for robotic applications 
Author: Yixi Cai
email: yixicai@connect.hku.hk
*/
extern float mid2min, planarity, angle_threshold;
extern int num_surfel, surfel_points_min;
const float flag_surfel = -9999;
VoxelInfo::VoxelInfo(const PointType &p) {
//    pthread_mutex_init(&updating_mutex_lock, NULL);
//    pthread_mutex_lock(&updating_mutex_lock);
    flag_updating = true;

    s_xx = p.x * p.x;
    s_xy = p.x * p.y;
    s_xz = p.x * p.z;
    s_yy = p.y * p.y;
    s_yz = p.y * p.z;
    s_zz = p.z * p.z;

    num_points = 1;
    mean_x = p.x;
    mean_y = p.y;
    mean_z = p.z;

    normal = p.getNormalVector3fMap();
//    cube_center(0) = voxel_center[0];
//    cube_center(1) = voxel_center[1];
//    cube_center(2) = voxel_center[2];
    points.push_back(p);
    is_init = true;

    plane = new Plane;
    flag_updating = false;
//    pthread_mutex_unlock(&updating_mutex_lock);
}

void VoxelInfo::getSimpInfo(SimpInfo& out) {
    out.s_xx = s_xx; out.s_yy = s_yy; out.s_zz = s_zz;
    out.s_xy = s_xy; out.s_xz = s_xz; out.s_yz = s_yz;

    out.normal = normal;
    out.mean_xyz(0) = mean_x;
    out.mean_xyz(1) = mean_y;
    out.mean_xyz(2) = mean_z;
}

void VoxelInfo::init() {
    num_points = 0;
    s_xx = 0.0; s_xy = 0.0; s_xz = 0.0;
    s_yy = 0.0; s_yz = 0.0; s_zz = 0.0;
    mean_x = 0.0; mean_y = 0.0; mean_z = 0.0;
    normal = Eigen::Vector3f::Zero();
    plane = new Plane;
    is_init = true;
}

void VoxelInfo::updateWithSingle(const PointType &p, const Eigen::Matrix3f & cov) {
//    pthread_mutex_lock(&updating_mutex_lock);
//    lock_guard<mutex> lock(updating_mutex_lock);
    flag_updating = true;
    updateSumAndMean(p);
    points.push_back(p);

    updateNormal(p);

    //check convergence
    if (num_points >= surfel_points_min) {
        checkPCA();
//    double t1 = omp_get_wtime();
        updatePlaneUncertainty(p, cov);
//    t1 = omp_get_wtime() - t1;
//    printf("updatePlaneUncertainty cost: %fs\n", t1);
    }
    flag_updating = false;
//    pthread_mutex_unlock(&updating_mutex_lock);
}

void VoxelInfo::updateWithSingle(const PointType &p) {
//    pthread_mutex_lock(&updating_mutex_lock);
//    lock_guard<mutex> lock(updating_mutex_lock);
    flag_updating = true;
    points.push_back(p);
    updateSumAndMean(p);
    updateNormal(p);

    if (num_points >= surfel_points_min)
        checkPCA();

    flag_updating = false;
//    pthread_mutex_unlock(&updating_mutex_lock);
}

void VoxelInfo::updateSumAndMean(const PointType & p) {
    // num of points in the voxel
    ++(num_points);

    // summation of p * p^t
    s_xx += p.x * p.x;
    s_xy += p.x * p.y;
    s_xz += p.x * p.z;
    s_yy += p.y * p.y;
    s_yz += p.y * p.z;
    s_zz += p.z * p.z;

    // summation of p
    float n = (float)num_points;
    mean_x = mean_x * (n - 1.0) / n + p.x / n;
    mean_y = mean_y * (n - 1.0) / n + p.y / n;
    mean_z = mean_z * (n - 1.0) / n + p.z / n;
}

void VoxelInfo::updatePlaneUncertainty(const PointType &p, const Eigen::Matrix3f & p_cov) {
    plane->plane_cov = Eigen::Matrix<float, 6, 6>::Zero();
    plane->points_size = points.size();

    // plane covariance calculation
    Eigen::Matrix3f J_Q; // dq/dp, q = mean_p
    J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0,
            1.0 / plane->points_size; //diag(1/n, 1/n, 1/n)
    // min eigen value small enough
//    if (evalsReal(evalsMin) < planer_threshold_) {
    if (num_points == flag_surfel) {
        vector<int> index(points.size());
        Eigen::Matrix3f U; // eigen vector
        U.block<3, 1>(0, 0) = plane->x_normal;
        U.block<3, 1>(0, 1) = plane->y_normal;
        U.block<3, 1>(0, 2) = plane->normal;
        // traverse all points
        for (int i = 0; i < points.size(); i++) {
            Eigen::Matrix<float, 6, 3> J; // 6 * 3
            Eigen::Matrix3f F;

            F.row(0) =
                    (points[i].getVector3fMap() - plane->center).transpose() /                     // (p - mean_p)^T /
                    ((plane->points_size) * (plane->min_eigen_value - plane->max_eigen_value)) *   // n * (lambda_3 - lambda_m) *
                    (plane->x_normal * plane->normal.transpose() +                                 // (u_m * n^T + n^T * u_m^T)
                    plane->normal * plane->x_normal.transpose());

            F.row(1) =
                    (points[i].getVector3fMap() - plane->center).transpose() /                     // (p - mean_p)^T /
                    ((plane->points_size) * (plane->min_eigen_value - plane->mid_eigen_value)) *   // n * (lambda_3 - lambda_m) *
                    (plane->y_normal * plane->normal.transpose() +                                 // (u_m * n^T + n^T * u_m^T)
                     plane->normal * plane->y_normal.transpose());

            F.row(2) << 0, 0, 0;

            J.block<3, 3>(0, 0) = U * F; // U * F, (dn/dp)
            J.block<3, 3>(3, 0) = J_Q; //J_Q, (dq/dp), q = mean_p
            plane->plane_cov += J * p_cov * J.transpose(); //cov (n, q), q = mean_p
        }

        plane->is_plane = true;
        if (plane->last_update_points_size == 0) {
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        } else if (plane->points_size - plane->last_update_points_size > 100) { // update interval: 100 points
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        }

        if (!plane->is_init) {
            plane->id = plane_id;
            plane_id++;
            plane->is_init = true;
        }

    } else { // min eigen value is not small enough, not yet a plane
        if (!plane->is_init) {
            plane->id = plane_id;
            plane_id++;
            plane->is_init = true;
        }
        if (plane->last_update_points_size == 0) {
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        } else if (plane->points_size - plane->last_update_points_size > 100) { // update interval: 100 points
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        }
        plane->is_plane = false;
    }
}

void VoxelInfo::updateNormal(const PointType &p) {
    // downsample normal
    normal = normal * (num_points - 1.0) / (float)num_points + p.getNormalVector3fMap() / (float)num_points;
    normal.normalize();
}

bool VoxelInfo::checkPCA() {
    const float n = (float)num_points;
    Eigen::Matrix3f cov;
    cov(0, 0) = s_xx / n - mean_x * mean_x;
    cov(1, 1) = s_yy / n - mean_y * mean_y;
    cov(2, 2) = s_zz / n - mean_z * mean_z;

    cov(0, 1) = s_xy / n - mean_x * mean_y;
    cov(1, 0) = cov(0, 1);

    cov(0, 2) = s_xz / n - mean_x * mean_z;
    cov(2, 0) = cov(0, 2);

    cov(1, 2) = s_yz / n - mean_y * mean_z;
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

    float normal_dot = eigenvectors_.col (2).dot(normal);
    float angle = acos(normal_dot); // 0 ~ pi
    angle = angle / M_PI * 180.0;
    if (normal_dot < 0) {
        eigenvectors_.col(2) *= -1.0;
        angle = 180.0 - angle;
    }

    ///planarity
    float pho = 2.0 * (eigenvalues_[1] - eigenvalues_[2]) / (eigenvalues_[0] + eigenvalues_[1] + eigenvalues_[2]);
    float mid_min = eigenvalues_[1] / eigenvalues_[2];

    if (mid_min > mid2min && pho > planarity) {
        if (num_points > 0) {
            num_points = flag_surfel; // mark surfel
            ++num_surfel;
        }

        if (angle < angle_threshold) {
            normal = (normal + eigenvectors_.col (2)) / 2.0; //todo
            normal.normalize();
        }
        else
            normal = eigenvectors_.col (2);

        //record parameters for plane
        plane->covariance = cov;
        plane->center << mean_x, mean_y, mean_z;
        plane->normal << eigenvectors_.col (2);
        plane->y_normal << eigenvectors_.col (1);
        plane->x_normal << eigenvectors_.col (0);
        plane->min_eigen_value = eigenvalues_[2];
        plane->mid_eigen_value = eigenvalues_[1];
        plane->max_eigen_value = eigenvalues_[0];
        plane->radius = plane->max_eigen_value;
        plane->d = -plane->normal.dot(plane->center);
        return true;
    }
    else
        return false;
}

template <typename PointType>
KD_TREE<PointType>::KD_TREE(float delete_param, float balance_param, float box_length) {
    delete_criterion_param = delete_param;
    balance_criterion_param = balance_param;
    downsample_size = box_length;
    Rebuild_Logger.clear();           
    termination_flag = false;
    start_thread();
}

template <typename PointType>
KD_TREE<PointType>::~KD_TREE()
{
    stop_thread();
    Delete_Storage_Disabled = true;
    delete_tree_nodes(&Root_Node);
    PointVector ().swap(PCL_Storage);
    Rebuild_Logger.clear();           
}

template <typename PointType>
void KD_TREE<PointType>::Set_delete_criterion_param(float delete_param){
    delete_criterion_param = delete_param;
}

template <typename PointType>
void KD_TREE<PointType>::Set_balance_criterion_param(float balance_param){
    balance_criterion_param = balance_param;
}

template <typename PointType>
void KD_TREE<PointType>::set_downsample_param(float downsample_param){
    downsample_size = downsample_param;
    half_voxel_size = 0.5 * downsample_size;
    quater_voxel_size = 0.5 * half_voxel_size;
    printf("half_voxel_size: %0.6f, quater_voxel_size: %0.6f\n", half_voxel_size, quater_voxel_size);
}

template <typename PointType>
void KD_TREE<PointType>::InitializeKDTree(float delete_param, float balance_param, float box_length){
    Set_delete_criterion_param(delete_param);
    Set_balance_criterion_param(balance_param);
    set_downsample_param(box_length);
}

template <typename PointType>
void KD_TREE<PointType>::InitTreeNode(KD_TREE_NODE * root){
    root->point.x = 0.0f;
    root->point.y = 0.0f;
    root->point.z = 0.0f;       
    root->point.normal_x = 0.0f;
    root->point.normal_y = 0.0f;
    root->point.normal_z = 0.0f;
    root->node_range_x[0] = 0.0f;
    root->node_range_x[1] = 0.0f;
    root->node_range_y[0] = 0.0f;
    root->node_range_y[1] = 0.0f;    
    root->node_range_z[0] = 0.0f;
    root->node_range_z[1] = 0.0f;     
    root->division_axis = 0;
    root->father_ptr = nullptr;
    root->left_son_ptr = nullptr;
    root->right_son_ptr = nullptr;
    root->TreeSize = 0;
    root->invalid_point_num = 0;
    root->down_del_num = 0;
    root->point_deleted = false;
    root->tree_deleted = false;
    root->need_push_down_to_left = false;
    root->need_push_down_to_right = false;
    root->point_downsample_deleted = false;
    root->working_flag = false;
    pthread_mutex_init(&(root->push_down_mutex_lock),NULL);
}   

template <typename PointType>
int KD_TREE<PointType>::size(){
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        if (Root_Node != nullptr) {
            return Root_Node->TreeSize;
        } else {
            return 0;
        }
    } else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            s = Root_Node->TreeSize;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        } else {
            return Treesize_tmp;
        }
    }
}

template <typename PointType>
BoxPointType KD_TREE<PointType>::tree_range(){
    BoxPointType range;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        if (Root_Node != nullptr) {
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
        } else {
            memset(&range, 0, sizeof(range));
        }
    } else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
            pthread_mutex_unlock(&working_flag_mutex);
        } else {
            memset(&range, 0, sizeof(range));
        }
    }
    return range;
}

template <typename PointType>
int KD_TREE<PointType>::validnum(){
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        if (Root_Node != nullptr)
            return (Root_Node->TreeSize - Root_Node->invalid_point_num);
        else 
            return 0;
    } else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            s = Root_Node->TreeSize-Root_Node->invalid_point_num;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        } else {
            return -1;
        }
    }
}

template <typename PointType>
void KD_TREE<PointType>::root_alpha(float &alpha_bal, float &alpha_del){
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        alpha_bal = Root_Node->alpha_bal;
        alpha_del = Root_Node->alpha_del;
        return;
    } else {
        if (!pthread_mutex_trylock(&working_flag_mutex)){
            alpha_bal = Root_Node->alpha_bal;
            alpha_del = Root_Node->alpha_del;
            pthread_mutex_unlock(&working_flag_mutex);
            return;
        } else {
            alpha_bal = alpha_bal_tmp;
            alpha_del = alpha_del_tmp;      
            return;
        }
    }    
}

template <typename PointType>
void KD_TREE<PointType>::start_thread(){
    pthread_mutex_init(&termination_flag_mutex_lock, NULL);   
    pthread_mutex_init(&rebuild_ptr_mutex_lock, NULL);     
    pthread_mutex_init(&rebuild_logger_mutex_lock, NULL);
    pthread_mutex_init(&points_deleted_rebuild_mutex_lock, NULL); 
    pthread_mutex_init(&working_flag_mutex, NULL);
    pthread_mutex_init(&search_flag_mutex, NULL);
    pthread_create(&rebuild_thread, NULL, multi_thread_ptr, (void*) this);
    printf("Multi thread started \n");    
}

template <typename PointType>
void KD_TREE<PointType>::stop_thread(){
    pthread_mutex_lock(&termination_flag_mutex_lock);
    termination_flag = true;
    pthread_mutex_unlock(&termination_flag_mutex_lock);
    if (rebuild_thread) pthread_join(rebuild_thread, NULL);
    pthread_mutex_destroy(&termination_flag_mutex_lock);
    pthread_mutex_destroy(&rebuild_logger_mutex_lock);
    pthread_mutex_destroy(&rebuild_ptr_mutex_lock);
    pthread_mutex_destroy(&points_deleted_rebuild_mutex_lock);
    pthread_mutex_destroy(&working_flag_mutex);
    pthread_mutex_destroy(&search_flag_mutex);     
}

template <typename PointType>
void * KD_TREE<PointType>::multi_thread_ptr(void * arg){
    KD_TREE * handle = (KD_TREE*) arg;
    handle->multi_thread_rebuild();
    return nullptr;    
}

template <typename PointType>
void KD_TREE<PointType>::multi_thread_rebuild(){
    bool terminated = false;
    KD_TREE_NODE * father_ptr, ** new_node_ptr;
    pthread_mutex_lock(&termination_flag_mutex_lock);
    terminated = termination_flag;
    pthread_mutex_unlock(&termination_flag_mutex_lock);
    while (!terminated){
        pthread_mutex_lock(&rebuild_ptr_mutex_lock);
        pthread_mutex_lock(&working_flag_mutex);
        if (Rebuild_Ptr != nullptr ){                    
            /* Traverse and copy */
            if (!Rebuild_Logger.empty()){
                printf("\n\n\n\n\n\n\n\n\n\n\n ERROR!!! \n\n\n\n\n\n\n\n\n");
            }
            rebuild_flag = true;
            if (*Rebuild_Ptr == Root_Node) {
                Treesize_tmp = Root_Node->TreeSize;
                Validnum_tmp = Root_Node->TreeSize - Root_Node->invalid_point_num;
                alpha_bal_tmp = Root_Node->alpha_bal;
                alpha_del_tmp = Root_Node->alpha_del;
            }
            KD_TREE_NODE * old_root_node = (*Rebuild_Ptr);                            
            father_ptr = (*Rebuild_Ptr)->father_ptr;  
            PointVector ().swap(Rebuild_PCL_Storage);
            // Lock Search 
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter != 0){
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);             
                pthread_mutex_lock(&search_flag_mutex);
            }
            search_mutex_counter = -1; //标记-1,执行flatten
            pthread_mutex_unlock(&search_flag_mutex);
            // Lock deleted points cache
            pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);    
            flatten(*Rebuild_Ptr, Rebuild_PCL_Storage, MULTI_THREAD_REC);  ///
            // Unlock deleted points cache
            pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);
            // Unlock Search
            pthread_mutex_lock(&search_flag_mutex); //search_flag_mutex上锁
            search_mutex_counter = 0;//标记为0
            pthread_mutex_unlock(&search_flag_mutex);              
            pthread_mutex_unlock(&working_flag_mutex);   
            /* Rebuild and update missed operations*/
            Operation_Logger_Type Operation;
            KD_TREE_NODE * new_root_node = nullptr;  
            if (int(Rebuild_PCL_Storage.size()) > 0){
                BuildTree(&new_root_node, 0, Rebuild_PCL_Storage.size()-1, Rebuild_PCL_Storage);
                // Rebuild has been done. Updates the blocked operations into the new tree
                pthread_mutex_lock(&working_flag_mutex);
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                int tmp_counter = 0;
                while (!Rebuild_Logger.empty()){
                    Operation = Rebuild_Logger.front();
                    max_queue_size = max(max_queue_size, Rebuild_Logger.size());
                    Rebuild_Logger.pop();
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);                  
                    pthread_mutex_unlock(&working_flag_mutex);
                    run_operation(&new_root_node, Operation);
                    tmp_counter ++;
                    if (tmp_counter % 10 == 0) usleep(1);
                    pthread_mutex_lock(&working_flag_mutex);
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);               
                }   
               pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }  
            /* Replace to original tree*/          
            // pthread_mutex_lock(&working_flag_mutex);
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter != 0){
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);             
                pthread_mutex_lock(&search_flag_mutex);
            }
            search_mutex_counter = -1; //标记-1，执行替换
            pthread_mutex_unlock(&search_flag_mutex);
            if (father_ptr->left_son_ptr == *Rebuild_Ptr) { //替换tree
                father_ptr->left_son_ptr = new_root_node;
            } else if (father_ptr->right_son_ptr == *Rebuild_Ptr){             
                father_ptr->right_son_ptr = new_root_node;
            } else {
                throw "Error: Father ptr incompatible with current node\n";
            }
            if (new_root_node != nullptr) new_root_node->father_ptr = father_ptr;
            (*Rebuild_Ptr) = new_root_node;
            int valid_old = old_root_node->TreeSize-old_root_node->invalid_point_num;
            int valid_new = new_root_node->TreeSize-new_root_node->invalid_point_num;
            if (father_ptr == STATIC_ROOT_NODE) Root_Node = STATIC_ROOT_NODE->left_son_ptr;
            KD_TREE_NODE * update_root = *Rebuild_Ptr;
            while (update_root != nullptr && update_root != Root_Node){
                update_root = update_root->father_ptr;
                if (update_root->working_flag) break;
                if (update_root == update_root->father_ptr->left_son_ptr && update_root->father_ptr->need_push_down_to_left) break;
                if (update_root == update_root->father_ptr->right_son_ptr && update_root->father_ptr->need_push_down_to_right) break;
                Update(update_root);
            }
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter = 0;
            pthread_mutex_unlock(&search_flag_mutex);
            Rebuild_Ptr = nullptr;
            pthread_mutex_unlock(&working_flag_mutex);
            rebuild_flag = false;                     
            /* Delete discarded tree nodes */
            delete_tree_nodes(&old_root_node);
            // todo delete pointer to corresponding voxel, !!caution: before now node setting

        } else {
            pthread_mutex_unlock(&working_flag_mutex);             
        }
        pthread_mutex_unlock(&rebuild_ptr_mutex_lock);         
        pthread_mutex_lock(&termination_flag_mutex_lock);
        terminated = termination_flag;
        pthread_mutex_unlock(&termination_flag_mutex_lock);
        usleep(100); 
    }
    printf("Rebuild thread terminated normally\n");    
}

template <typename PointType>
void KD_TREE<PointType>::run_operation(KD_TREE_NODE ** root, Operation_Logger_Type operation){
    switch (operation.op)
    {
    case ADD_POINT:      
        Add_by_point(root, operation.point, false, (*root)->division_axis);          
        break;
    case ADD_BOX:
        Add_by_range(root, operation.boxpoint, false);
        break;
    case DELETE_POINT:
        Delete_by_point(root, operation.point, false);
        break;
    case DELETE_BOX:
        Delete_by_range(root, operation.boxpoint, false, false);
        break;
    case DOWNSAMPLE_DELETE:
        Delete_by_range(root, operation.boxpoint, false, true);
        break;
    case PUSH_DOWN:
        (*root)->tree_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->point_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->tree_deleted = operation.tree_deleted || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = (*root)->tree_deleted || (*root)->point_downsample_deleted;
        if (operation.tree_downsample_deleted) (*root)->down_del_num = (*root)->TreeSize;
        if (operation.tree_deleted) (*root)->invalid_point_num = (*root)->TreeSize;
            else (*root)->invalid_point_num = (*root)->down_del_num;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;     
        break;
    default:
        break;
    }
}

template <typename PointType>
void KD_TREE<PointType>::insideVoxelDownsample(PointVector& point_cloud) {
//    voxel2node.clear();
//    node2voxel.clear();
    unordered_map<VOXEL_LOC, PointType> voxel2point;

    for (PointType& p : point_cloud)
    {
//        const PointType& p = root->point;
        float loc_xyz[3];
        point2Voxel(p, loc_xyz);
        VOXEL_LOC voxel = VOXEL_LOC((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
//        VOXEL_LOC voxel;
//        point2Voxel(p, voxel);
//        auto iter = voxel2node.find(voxel);
        if (voxel2point.find(voxel) == voxel2point.end()) {
            //already have the voexl
            voxel2point[voxel] = p;
//            node2voxel[root] = voxel;
//            voxel_map[voxel]->temp_points_.push_back(p_v);
//            voxel_map[voxel]->new_points_num_++;
        }
        else {
            const PointType& p_old = voxel2point[voxel];
            PointType mid_point;
            mid_point.x = loc_xyz[0];
            mid_point.y = loc_xyz[1];
            mid_point.z = loc_xyz[2];
            float dist_old = calc_dist(p_old, mid_point);
            float dist_new = calc_dist(p, mid_point);
            if (dist_new < dist_old)
            {
                voxel2point[voxel] = p;
            }
        }
    }

    int i = 0;
    for (const auto& n : voxel2point)
        point_cloud[i++] = n.second;
    point_cloud.resize(i);
}

template <typename PointType>
void KD_TREE<PointType>::Build(PointVector point_cloud){
    if (Root_Node != nullptr){
        delete_tree_nodes(&Root_Node);
    }
    if (point_cloud.size() == 0) return;

    //todo voxel grid downsample first
//    insideVoxelDownsample(point_cloud);

    STATIC_ROOT_NODE = new KD_TREE_NODE;
    InitTreeNode(STATIC_ROOT_NODE);
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size()-1, point_cloud);
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->TreeSize = 0;
    Root_Node = STATIC_ROOT_NODE->left_son_ptr;

    // todo build connection between voxels and nodes
    buildConnection(Root_Node);
}

template <typename PointType>
void KD_TREE<PointType>::Nearest_Search(PointType point, int k_nearest, PointVector& Nearest_Points, vector<float> & Point_Distance, double max_dist){   
    MANUAL_HEAP q(2*k_nearest);
    q.clear();
    vector<float> ().swap(Point_Distance);
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
        Search(Root_Node, k_nearest, point, q, max_dist);
    } else {
        pthread_mutex_lock(&search_flag_mutex);
        while (search_mutex_counter == -1)
        {
            pthread_mutex_unlock(&search_flag_mutex);
            usleep(1);
            pthread_mutex_lock(&search_flag_mutex);
        }
        search_mutex_counter += 1;
        pthread_mutex_unlock(&search_flag_mutex);  
        Search(Root_Node, k_nearest, point, q, max_dist);  
        pthread_mutex_lock(&search_flag_mutex);
        search_mutex_counter -= 1;
        pthread_mutex_unlock(&search_flag_mutex);      
    }
    int k_found = min(k_nearest,int(q.size()));
    PointVector ().swap(Nearest_Points);
    vector<float> ().swap(Point_Distance);
    for (int i=0;i < k_found;i++){
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }
    return;
}

int six_neighbor[6] = {1, -1, 1, -1, 1, -1}; // dx, dy, dz
template <typename PointType>
void KD_TREE<PointType>::addNearVoxelsLoc(const PointType& point, const float* voxel_center, std::vector<float>& voxels)
{
//    int dimension = 0;
//    vector<bool> six_neighbor({1, -1, 1, -1, 1, -1});
    //2-4-8 neighbors
    for (int dimension = 0; dimension < 3; ++dimension) {
        int d_dimension = 0;
        const float & coor = point.getVector3fMap()[dimension];
        if (coor > (voxel_center[dimension] + quater_voxel_size)) {
            d_dimension = 1;
            six_neighbor[dimension * 2] = 0;
        }
        else if (coor < (voxel_center[dimension] - quater_voxel_size)) {
            d_dimension = -1;
            six_neighbor[dimension * 2 + 1] = 0;
        }

        if (d_dimension == 0)
            continue;
        // 2-4-8 neighbors first
        int size_old = voxels.size();
        voxels.resize(size_old * 2);
        for (int i = 0; i < size_old / 3; ++i) {
            voxels[i * 3 + size_old] = voxels[i * 3];
            voxels[i * 3 + 1 + size_old] = voxels[i * 3 + 1];
            voxels[i * 3 + 2 + size_old] = voxels[i * 3 + 2];
            voxels[i * 3 + size_old + dimension] += d_dimension;
        }
    }

    // 6 nearest neighbors
    int size_old = voxels.size();
    for (int i = 0; i < 6; ++i) {
        if (six_neighbor[i] == 0)
            continue;
        int dimension = i / 2;
        voxels.emplace_back(voxels[0]);
        voxels.emplace_back(voxels[1]);
        voxels.emplace_back(voxels[2]);
        voxels[size_old + dimension] += six_neighbor[i];
        size_old += 3;
    }
}

template <typename PointType>
void KD_TREE<PointType>::updateVoxelInfo(const PointType& point, const Eigen::Matrix3f & cov)
{
    VOXEL_LOC voxel;
//    float cube_center[3];
    point2Voxel(point, voxel);
    auto iter = voxel2info.find(voxel);
    VoxelInfoPtr voxel_ptr;
    if (iter == voxel2info.end() || !iter->second) //for voxel that not initialize yet
    {
        voxel_ptr = new VoxelInfo(point);
        voxel2info[voxel] = voxel_ptr;
        return;
    }
    else
        voxel_ptr = iter->second;

    if (voxel_ptr->num_points > 0 && voxel_ptr->num_points < max_voxel_points_size) {
//        printf("num_points %d, surfel_points_min %d, max_voxel_points_size: %d.\n", voxel_ptr->num_points,
//               surfel_points_min, max_voxel_points_size);
        voxel_ptr->updateWithSingle(point, cov);
    }
}

template <typename PointType>
void KD_TREE<PointType>::updateVoxelInfo(const PointType& point)
{
    VOXEL_LOC voxel;
//    float cube_center[3];
    point2Voxel(point, voxel);
    auto iter = voxel2info.find(voxel);
    VoxelInfoPtr voxel_ptr;
    if (iter == voxel2info.end() || !iter->second) //for voxel that not initialize yet
    {
        voxel_ptr = new VoxelInfo(point);
        voxel2info[voxel] = voxel_ptr;
        return;
    }
    else
        voxel_ptr = iter->second;

    if (voxel_ptr->num_points > 0 && voxel_ptr->num_points < max_voxel_points_size)
        voxel_ptr->updateWithSingle(point);
}

template <typename PointType>
void KD_TREE<PointType>::Voxel_Search_test(const PointType& point, int k_nearest, PointVector &Nearest_Points, vector<float> & Point_Distance,  vector<VoxelInfoPtr>& voxels)
{
    PointVector().swap(Nearest_Points);
    vector<VoxelInfoPtr>().swap(voxels);
    vector<float> ().swap(Point_Distance);

    //divide into voxel
    float loc_xyz[3], center_xyz[3];
    point2Voxel(point, loc_xyz, center_xyz);
//    printf("\npoint: %f, %f, %f loc_xyz: %f, %f, %f center_xyz: %f, %f, %f.\n", point.x, point.y, point.z,
//           loc_xyz[0], loc_xyz[1], loc_xyz[2],
//           center_xyz[0], center_xyz[1], center_xyz[2]);

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    vector<float> near_voxel_loc;
    near_voxel_loc.reserve(30);
    // record first voxel index;
    near_voxel_loc.emplace_back(loc_xyz[0]);
    near_voxel_loc.emplace_back(loc_xyz[1]);
    near_voxel_loc.emplace_back(loc_xyz[2]);

    addNearVoxelsLoc(point, center_xyz, near_voxel_loc);

    MANUAL_VOXEL_HEAP q(k_nearest);
    q.clear();
    for (int i = 0; i < near_voxel_loc.size() / 3; ++i) {
//        printf("near_loc_xyz: %f, %f, %f\n", near_voxel_loc[i * 3], near_voxel_loc[i * 3 + 1], near_voxel_loc[i * 3 + 2]);
        VOXEL_LOC near_position((int64_t)near_voxel_loc[i * 3], (int64_t)near_voxel_loc[i * 3 + 1], near_voxel_loc[i * 3 + 2]);
//        const auto & near_iter = voxel2node.find(near_position);
        const auto & near_iter = voxel2info.find(near_position);
        if (near_iter != voxel2info.end()) {
            VoxelInfo* voxel_info = near_iter->second;
//            const PointType& p = near_iter->second->point;
//            float dist = calc_dist(p, point);

            if (voxel_info) {
                PointType p;
//                while (voxel_info->flag_updating) /// !!!caution
//                    usleep(2);
                lock_guard<mutex> lock(voxel_info->updating_mutex_lock);
//                pthread_mutex_lock(&voxel_info->updating_mutex_lock);
                p.x = voxel_info->mean_x; p.y = voxel_info->mean_y; p.z = voxel_info->mean_z;
//                pthread_mutex_unlock(&voxel_info->updating_mutex_lock);
                float dist = calc_dist(point, p);
//                if (dist <= max_dist_sqr && (q.size() < k_nearest || dist < q.top().dist)){
                if (q.size() < k_nearest || dist < q.top().dist) {
                    if (q.size() >= k_nearest) q.pop();
                    PointType_Voxel_CMP current_point{p, voxel_info, dist};
                    q.push(current_point);
                }
            }
        }
    }

    int k_found = min(k_nearest,int(q.size()));
    for (int i=0;i < k_found;i++){
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        voxels.insert(voxels.begin(), q.top().info);
//        q.top().info->getSimpInfo(s_info);
        q.pop();
    }
}

template <typename PointType>
void KD_TREE<PointType>::Voxel_Search(const PointType& point, int k_nearest, PointVector &Nearest_Points, vector<KD_TREE_NODE*>& nodes)
{
    //divide into voxel
    float loc_xyz[3], center_xyz[3];
    point2Voxel(point, loc_xyz, center_xyz);
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    const auto & iter = voxel2node.find(position);
    if (iter == voxel2node.end())
        return;
    KD_TREE_NODE* node_center = iter->second;
    nodes.emplace_back(node_center);
    Nearest_Points.push_back(node_center->point);

    VOXEL_LOC near_position = position;
}


template <typename PointType>
void KD_TREE<PointType>::Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points,
                                         vector<float> & Point_Distance, vector<VoxelInfoPtr>& voxels, double max_dist)
{
    MANUAL_HEAP q(2*k_nearest);
    q.clear();
    vector<float> ().swap(Point_Distance);
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){        // 没有rebuild,直接search
        Search(Root_Node, k_nearest, point, q, max_dist);
    } else {
        // 如果有rebuild
        pthread_mutex_lock(&search_flag_mutex); //search_flag_mutex
        while (search_mutex_counter == -1)
        {
            pthread_mutex_unlock(&search_flag_mutex);//search_flag_mutex
            usleep(1);
            pthread_mutex_lock(&search_flag_mutex);//search_flag_mutex
        }
        search_mutex_counter += 1; // +1?
        pthread_mutex_unlock(&search_flag_mutex);
        Search(Root_Node, k_nearest, point, q, max_dist);
        pthread_mutex_lock(&search_flag_mutex);
        search_mutex_counter -= 1;
        pthread_mutex_unlock(&search_flag_mutex);
    }
    int k_found = min(k_nearest,int(q.size()));
    PointVector ().swap(Nearest_Points);
    vector<float> ().swap(Point_Distance);
    vector<VoxelInfoPtr> ().swap(voxels);
    for (int i=0;i < k_found;i++){
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        VOXEL_LOC voxel;
        point2Voxel(q.top().point, voxel);

        auto iter = voxel2info.find(voxel);
        VoxelInfoPtr voxel_ptr;
        if (iter == voxel2info.end() || !iter->second) //for voxel that not initialize yet
        {
            voxel_ptr = new VoxelInfo(point);
            voxel2info[voxel] = voxel_ptr;
            return;
        }
        else
            voxel_ptr = iter->second;

        voxels.insert(voxels.begin(), voxel_ptr);
        q.pop();
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Box_Search(const BoxPointType &Box_of_Point, PointVector &Storage)
{
    Storage.clear();
    Search_by_range(Root_Node, Box_of_Point, Storage);
}

template <typename PointType>
void KD_TREE<PointType>::Radius_Search(PointType point, const float radius, PointVector &Storage)
{
    Storage.clear();
    Search_by_radius(Root_Node, point, radius, Storage);
}

template <typename PointType>
int KD_TREE<PointType>::Add_Points(PointVector & PointToAdd, bool downsample_on){
    int NewPointSize = PointToAdd.size();
    int tree_size = size();
    BoxPointType Box_of_Point;
    PointType downsample_result, mid_point;
    bool downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
    float min_dist, tmp_dist;
    int tmp_counter = 0;
    for (int i=0; i<PointToAdd.size();i++){
        if (downsample_switch){
            Box_of_Point.vertex_min[0] = floor(PointToAdd[i].x/downsample_size)*downsample_size;
            Box_of_Point.vertex_max[0] = Box_of_Point.vertex_min[0]+downsample_size;
            Box_of_Point.vertex_min[1] = floor(PointToAdd[i].y/downsample_size)*downsample_size;
            Box_of_Point.vertex_max[1] = Box_of_Point.vertex_min[1]+downsample_size; 
            Box_of_Point.vertex_min[2] = floor(PointToAdd[i].z/downsample_size)*downsample_size;
            Box_of_Point.vertex_max[2] = Box_of_Point.vertex_min[2]+downsample_size;   
            mid_point.x = Box_of_Point.vertex_min[0] + (Box_of_Point.vertex_max[0]-Box_of_Point.vertex_min[0])/2.0;
            mid_point.y = Box_of_Point.vertex_min[1] + (Box_of_Point.vertex_max[1]-Box_of_Point.vertex_min[1])/2.0;
            mid_point.z = Box_of_Point.vertex_min[2] + (Box_of_Point.vertex_max[2]-Box_of_Point.vertex_min[2])/2.0;
            PointVector ().swap(Downsample_Storage);
            Search_by_range(Root_Node, Box_of_Point, Downsample_Storage);
            min_dist = calc_dist(PointToAdd[i],mid_point);
            downsample_result = PointToAdd[i]; // point to be added
            for (int index = 0; index < Downsample_Storage.size(); index++){
                tmp_dist = calc_dist(Downsample_Storage[index], mid_point);
                if (tmp_dist < min_dist){
                    min_dist = tmp_dist;
                    downsample_result = Downsample_Storage[index];
                }
            }
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){  
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result)){
                    if (Downsample_Storage.size() > 0) Delete_by_range(&Root_Node, Box_of_Point, true, true);
                    Add_by_point(&Root_Node, downsample_result, true, Root_Node->division_axis);
                    tmp_counter ++;                      
                }
            } else {
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result)){
                    Operation_Logger_Type  operation_delete, operation;
                    operation_delete.boxpoint = Box_of_Point;
                    operation_delete.op = DOWNSAMPLE_DELETE;
                    operation.point = downsample_result;
                    operation.op = ADD_POINT;
                    pthread_mutex_lock(&working_flag_mutex);
                    if (Downsample_Storage.size() > 0) Delete_by_range(&Root_Node, Box_of_Point, false , true);                                      
                    Add_by_point(&Root_Node, downsample_result, false, Root_Node->division_axis);
                    tmp_counter ++;
                    if (rebuild_flag){
                        pthread_mutex_lock(&rebuild_logger_mutex_lock);
                        if (Downsample_Storage.size() > 0) Rebuild_Logger.push(operation_delete);
                        Rebuild_Logger.push(operation);
                        pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    }
                    pthread_mutex_unlock(&working_flag_mutex);
                };
            }
        } else {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
                Add_by_point(&Root_Node, PointToAdd[i], true, Root_Node->division_axis);     
            } else {
                Operation_Logger_Type operation;
                operation.point = PointToAdd[i];
                operation.op = ADD_POINT;                
                pthread_mutex_lock(&working_flag_mutex);
                Add_by_point(&Root_Node, PointToAdd[i], false, Root_Node->division_axis);
                if (rebuild_flag){
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                    Rebuild_Logger.push(operation);
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                }
                pthread_mutex_unlock(&working_flag_mutex);       
            }
        }
    }
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Add_Point_Boxes(vector<BoxPointType> & BoxPoints){     
    for (int i=0;i < BoxPoints.size();i++){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){
            Add_by_range(&Root_Node ,BoxPoints[i], true);
        } else {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = ADD_BOX;
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_range(&Root_Node ,BoxPoints[i], false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }               
            pthread_mutex_unlock(&working_flag_mutex);
        }    
    } 
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Delete_Points(PointVector & PointToDel){        
    for (int i=0;i<PointToDel.size();i++){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){               
            Delete_by_point(&Root_Node, PointToDel[i], true);
        } else {
            Operation_Logger_Type operation;
            operation.point = PointToDel[i];
            operation.op = DELETE_POINT;
            pthread_mutex_lock(&working_flag_mutex);        
            Delete_by_point(&Root_Node, PointToDel[i], false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }      
    }      
    return;
}

template <typename PointType>
int KD_TREE<PointType>::Delete_Point_Boxes(vector<BoxPointType> & BoxPoints){
    int tmp_counter = 0;
    for (int i=0;i < BoxPoints.size();i++){ 
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node){               
            tmp_counter += Delete_by_range(&Root_Node ,BoxPoints[i], true, false);
        } else {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = DELETE_BOX;     
            pthread_mutex_lock(&working_flag_mutex); 
            tmp_counter += Delete_by_range(&Root_Node ,BoxPoints[i], false, false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }                
            pthread_mutex_unlock(&working_flag_mutex);
        }
    } 
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::acquire_removed_points(PointVector & removed_points){
    pthread_mutex_lock(&points_deleted_rebuild_mutex_lock); 
    for (int i = 0; i < Points_deleted.size();i++){
        removed_points.push_back(Points_deleted[i]);
    }
    for (int i = 0; i < Multithread_Points_deleted.size();i++){
        removed_points.push_back(Multithread_Points_deleted[i]);
    }
    Points_deleted.clear();
    Multithread_Points_deleted.clear();
    pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);   
    return;
}

template <typename PointType>
void KD_TREE<PointType>::findVoxel(const PointType& p)
{
//    //divide into voxel
//    float loc_xyz[3];
//    loc_xyz[0] = p.x / downsample_size;
//    loc_xyz[1] = p.y / downsample_size;
//    loc_xyz[2] = p.z / downsample_size;
//    for (int j = 0; j < 3; j++)
//        if (loc_xyz[j] < 0)
//            loc_xyz[j] -= 1.0;
//
//    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
//    auto iter = voxel_map.find(position);
//
//    pointWithCov p_v;
//    p_v.point_world << p.x, p.y, p.z;
//    p.getPointCovMatrix(p_v.cov);
//    if (iter != voxel_map.end()) {
//        //already have the voexl
//        voxel_map[position]->temp_points_.push_back(p_v);
//        voxel_map[position]->new_points_num_++;
//    } else {
//        OctoTree *octo_tree =
//                new OctoTree(max_layer, 0, layer_point_size, max_points_size,
//                             max_cov_points_size, planer_threshold);
//        voxel_map[position] = octo_tree;
//        voxel_map[position]->quater_length_ = voxel_size / 4;
//        voxel_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
//        voxel_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
//        voxel_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
//        voxel_map[position]->temp_points_.push_back(p_v);
//        voxel_map[position]->new_points_num_++;
//        voxel_map[position]->layer_point_size_ = layer_point_size;
//    }
}

template <typename PointType>
void KD_TREE<PointType>::BuildTree(KD_TREE_NODE ** root, int l, int r, PointVector & Storage){
    if (l>r) return;
    *root = new KD_TREE_NODE;
    InitTreeNode(*root);
    int mid = (l+r)>>1;
    int div_axis = 0;
    int i;
    // Find the best division Axis
    float min_value[3] = {INFINITY, INFINITY, INFINITY};
    float max_value[3] = {-INFINITY, -INFINITY, -INFINITY};
    float dim_range[3] = {0,0,0};
    for (i=l;i<=r;i++){
        min_value[0] = min(min_value[0], Storage[i].x);
        min_value[1] = min(min_value[1], Storage[i].y);
        min_value[2] = min(min_value[2], Storage[i].z);
        max_value[0] = max(max_value[0], Storage[i].x);
        max_value[1] = max(max_value[1], Storage[i].y);
        max_value[2] = max(max_value[2], Storage[i].z);
    }
    // Select the longest dimension as division axis
    for (i=0;i<3;i++) dim_range[i] = max_value[i] - min_value[i];
    for (i=1;i<3;i++) if (dim_range[i] > dim_range[div_axis]) div_axis = i;
    // Divide by the division axis and recursively build.

    (*root)->division_axis = div_axis;
    switch (div_axis)
    {
    case 0:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_x);
        break;
    case 1:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_y);
        break;
    case 2:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_z);
        break;
    default:
        nth_element(begin(Storage)+l, begin(Storage)+mid, begin(Storage)+r+1, point_cmp_x);
        break;
    }  
    (*root)->point = Storage[mid];

    // connect to voxel
    single_connect(*root);

    KD_TREE_NODE * left_son = nullptr, * right_son = nullptr;
    BuildTree(&left_son, l, mid-1, Storage);
    BuildTree(&right_son, mid+1, r, Storage);  
    (*root)->left_son_ptr = left_son;
    (*root)->right_son_ptr = right_son;
    Update((*root));  
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Rebuild(KD_TREE_NODE ** root){    
    KD_TREE_NODE * father_ptr;
    if ((*root)->TreeSize >= Multi_Thread_Rebuild_Point_Num) { 
        if (!pthread_mutex_trylock(&rebuild_ptr_mutex_lock)){     
            if (Rebuild_Ptr == nullptr || ((*root)->TreeSize > (*Rebuild_Ptr)->TreeSize)) {
                Rebuild_Ptr = root;          
            }
            pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
        }
    } else {
        father_ptr = (*root)->father_ptr;
        int size_rec = (*root)->TreeSize;
        PCL_Storage.clear();
        flatten(*root, PCL_Storage, DELETE_POINTS_REC);
        delete_tree_nodes(root);
        // todo delete pointer to corresponding voxel
        BuildTree(root, 0, PCL_Storage.size()-1, PCL_Storage);
        if (*root != nullptr) (*root)->father_ptr = father_ptr;
        if (*root == Root_Node) STATIC_ROOT_NODE->left_son_ptr = *root;
    } 
    return;
}

template <typename PointType>
int KD_TREE<PointType>::Delete_by_range(KD_TREE_NODE ** root,  BoxPointType boxpoint, bool allow_rebuild, bool is_downsample){   
    if ((*root) == nullptr || (*root)->tree_deleted) return 0;
    (*root)->working_flag = true;
    Push_Down(*root);
    int tmp_counter = 0;
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1]) return 0;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1]) return 0;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1]) return 0;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] && boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1]){
        (*root)->tree_deleted = true;
        (*root)->point_deleted = true;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        tmp_counter = (*root)->TreeSize - (*root)->invalid_point_num;
        (*root)->invalid_point_num = (*root)->TreeSize;
        if (is_downsample){
            (*root)->tree_downsample_deleted = true;
            (*root)->point_downsample_deleted = true;
            (*root)->down_del_num = (*root)->TreeSize;
        }

        return tmp_counter;
    }
    if (!(*root)->point_deleted && boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z){
        (*root)->point_deleted = true;
        tmp_counter += 1;
        if (is_downsample) (*root)->point_downsample_deleted = true;
    }
    Operation_Logger_Type delete_box_log;
    struct timespec Timeout;    
    if (is_downsample) delete_box_log.op = DOWNSAMPLE_DELETE;
        else delete_box_log.op = DELETE_BOX;
    delete_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild, is_downsample);
    } else {
        pthread_mutex_lock(&working_flag_mutex);
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild, is_downsample);
    } else {
        pthread_mutex_lock(&working_flag_mutex);
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }    
    Update(*root);     
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    if ((*root) != nullptr) (*root)->working_flag = false;
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Delete_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild){   
    if ((*root) == nullptr || (*root)->tree_deleted) return;
    (*root)->working_flag = true;
    Push_Down(*root);
    if (same_point((*root)->point, point) && !(*root)->point_deleted) {          
        (*root)->point_deleted = true;
        (*root)->invalid_point_num += 1;
        if ((*root)->invalid_point_num == (*root)->TreeSize) (*root)->tree_deleted = true;
        return;
    }
    Operation_Logger_Type delete_log;
    struct timespec Timeout;    
    delete_log.op = DELETE_POINT;
    delete_log.point = point;     
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z)){           
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){          
            Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);         
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&(*root)->left_son_ptr, point,false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    } else {       
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){         
            Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);         
        } else {
            pthread_mutex_lock(&working_flag_mutex); 
            Delete_by_point(&(*root)->right_son_ptr, point, false);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }        
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    if ((*root) != nullptr) (*root)->working_flag = false;   
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Add_by_range(KD_TREE_NODE ** root, BoxPointType boxpoint, bool allow_rebuild){
    if ((*root) == nullptr) return;
    (*root)->working_flag = true;
    Push_Down(*root);       
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1]) return;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1]> (*root)->node_range_y[1] && boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1]){
        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = (*root)->down_del_num;

        // todo set pointer to corresponding voxel

        return;
    }
    if (boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z){
        (*root)->point_deleted = (*root)->point_downsample_deleted;
    }
    Operation_Logger_Type add_box_log;
    struct timespec Timeout;    
    add_box_log.op = ADD_BOX;
    add_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){
        Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
    } else {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->left_son_ptr), boxpoint, false);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }        
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){
        Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
    } else {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->right_son_ptr), boxpoint, false);
        if (rebuild_flag){
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root);
    if ((*root) != nullptr) (*root)->working_flag = false;   
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Add_by_point(KD_TREE_NODE ** root, PointType point, bool allow_rebuild, int father_axis){     
    if (*root == nullptr){
        *root = new KD_TREE_NODE;
        InitTreeNode(*root);
        (*root)->point = point;
        single_connect(*root);
        (*root)->division_axis = (father_axis + 1) % 3;
        Update(*root);
        return;
    }
    (*root)->working_flag = true;
    Operation_Logger_Type add_log;
    struct timespec Timeout;    
    add_log.op = ADD_POINT;
    add_log.point = point;
    Push_Down(*root);
    // left or right son
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z)){
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr){
            Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild, (*root)->division_axis);
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->left_son_ptr, point, false,(*root)->division_axis);
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex);            
        }
    } else {  
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr){         
            Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild,(*root)->division_axis);
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->right_son_ptr, point, false,(*root)->division_axis);       
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);                 
            }
            pthread_mutex_unlock(&working_flag_mutex); 
        }
    }
    Update(*root);   
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) Rebuild_Ptr = nullptr; 
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild) Rebuild(root); 
    if ((*root) != nullptr) (*root)->working_flag = false;   
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Search(KD_TREE_NODE * root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist){
    if (root == nullptr || root->tree_deleted) return;   
    double cur_dist = calc_box_dist(root, point);
    double max_dist_sqr = max_dist * max_dist;
    if (cur_dist > max_dist_sqr) return;    
    int retval; 
    if (root->need_push_down_to_left || root->need_push_down_to_right) {
        retval = pthread_mutex_trylock(&(root->push_down_mutex_lock));
        if (retval == 0){
            Push_Down(root);
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        } else {
            pthread_mutex_lock(&(root->push_down_mutex_lock));
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
    }

    if (!root->point_deleted){
        float dist = calc_dist(point, root->point);
        if (dist <= max_dist_sqr && (q.size() < k_nearest || dist < q.top().dist)){
            if (q.size() >= k_nearest) q.pop();
            PointType_CMP current_point{root->point, root, dist};
            q.push(current_point);            
        }
    }  
    int cur_search_counter;
    float dist_left_node = calc_box_dist(root->left_son_ptr, point);
    float dist_right_node = calc_box_dist(root->right_son_ptr, point);
    if (q.size()< k_nearest || dist_left_node < q.top().dist && dist_right_node < q.top().dist){
        if (dist_left_node <= dist_right_node) {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);  
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_right_node < q.top().dist) {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);                       
                } else {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);                    
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);  
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }                
            }
        } else {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);                   
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);  
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_left_node < q.top().dist) {            
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);                       
                } else {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);  
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);  
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
    } else {
        if (dist_left_node < q.top().dist) {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);  
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);  
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
        if (dist_right_node < q.top().dist) {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);                       
            } else {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);  
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector & Storage){
    if (root == nullptr) return;
    Push_Down(root);       
    if (boxpoint.vertex_max[0] <= root->node_range_x[0] || boxpoint.vertex_min[0] > root->node_range_x[1]) return;
    if (boxpoint.vertex_max[1] <= root->node_range_y[0] || boxpoint.vertex_min[1] > root->node_range_y[1]) return;
    if (boxpoint.vertex_max[2] <= root->node_range_z[0] || boxpoint.vertex_min[2] > root->node_range_z[1]) return;
    if (boxpoint.vertex_min[0] <= root->node_range_x[0] && boxpoint.vertex_max[0] > root->node_range_x[1] && boxpoint.vertex_min[1] <= root->node_range_y[0] && boxpoint.vertex_max[1] > root->node_range_y[1] && boxpoint.vertex_min[2] <= root->node_range_z[0] && boxpoint.vertex_max[2] > root->node_range_z[1]){
        flatten(root, Storage, NOT_RECORD);
        return;
    }
    if (boxpoint.vertex_min[0] <= root->point.x && boxpoint.vertex_max[0] > root->point.x && boxpoint.vertex_min[1] <= root->point.y && boxpoint.vertex_max[1] > root->point.y && boxpoint.vertex_min[2] <= root->point.z && boxpoint.vertex_max[2] > root->point.z){
        if (!root->point_deleted) Storage.push_back(root->point);
    }
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr){
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
    } else {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr){
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
    } else {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    return;    
}

template <typename PointType>
void KD_TREE<PointType>::Search_by_radius(KD_TREE_NODE *root, PointType point, float radius, PointVector &Storage)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    PointType range_center;
    range_center.x = (root->node_range_x[0] + root->node_range_x[1]) * 0.5;
    range_center.y = (root->node_range_y[0] + root->node_range_y[1]) * 0.5;
    range_center.z = (root->node_range_z[0] + root->node_range_z[1]) * 0.5;
    float dist = sqrt(calc_dist(range_center, point));
    if (dist > radius + sqrt(root->radius_sq)) return;
    if (dist <= radius - sqrt(root->radius_sq)) 
    {
        flatten(root, Storage, NOT_RECORD);
        return;
    }
    if (!root->point_deleted && calc_dist(root->point, point) <= radius * radius){
        Storage.push_back(root->point);
    }
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr)
    {
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr)
    {
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }    
    return;
}

template <typename PointType>
bool KD_TREE<PointType>::Criterion_Check(KD_TREE_NODE * root){
    if (root->TreeSize <= Minimal_Unbalanced_Tree_Size){
        return false;
    }
    float balance_evaluation = 0.0f;
    float delete_evaluation = 0.0f;
    KD_TREE_NODE * son_ptr = root->left_son_ptr;
    if (son_ptr == nullptr) son_ptr = root->right_son_ptr;
    delete_evaluation = float(root->invalid_point_num)/ root->TreeSize;
    balance_evaluation = float(son_ptr->TreeSize) / (root->TreeSize-1);  
    if (delete_evaluation > delete_criterion_param){
        return true;
    }
    if (balance_evaluation > balance_criterion_param || balance_evaluation < 1-balance_criterion_param){
        return true;
    } 
    return false;
}

template <typename PointType>
void KD_TREE<PointType>::Push_Down(KD_TREE_NODE *root){
    if (root == nullptr) return;
    Operation_Logger_Type operation;
    operation.op = PUSH_DOWN;
    operation.tree_deleted = root->tree_deleted;
    operation.tree_downsample_deleted = root->tree_downsample_deleted;
    if (root->need_push_down_to_left && root->left_son_ptr != nullptr){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr){
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted) root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted) root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
                else root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_left = false;                
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted) root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted) root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
                else root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;            
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            root->need_push_down_to_left = false;
            pthread_mutex_unlock(&working_flag_mutex);            
        }
    }
    if (root->need_push_down_to_right && root->right_son_ptr != nullptr){
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr){
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted) root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted) root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
                else root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_right = false;
        } else {
            pthread_mutex_lock(&working_flag_mutex);
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted) root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted) root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
                else root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;            
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag){
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }            
            root->need_push_down_to_right = false;
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Update(KD_TREE_NODE * root){
    KD_TREE_NODE * left_son_ptr = root->left_son_ptr;
    KD_TREE_NODE * right_son_ptr = root->right_son_ptr;
    float tmp_range_x[2] = {INFINITY, -INFINITY};
    float tmp_range_y[2] = {INFINITY, -INFINITY};
    float tmp_range_z[2] = {INFINITY, -INFINITY};
    // Update Tree Size   
    if (left_son_ptr != nullptr && right_son_ptr != nullptr){
        root->TreeSize = left_son_ptr->TreeSize + right_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + right_son_ptr->invalid_point_num + (root->point_deleted? 1:0);
        root->down_del_num = left_son_ptr->down_del_num + right_son_ptr->down_del_num + (root->point_downsample_deleted? 1:0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !right_son_ptr->tree_deleted && !root->point_deleted)){
            tmp_range_x[0] = min(min(left_son_ptr->node_range_x[0],right_son_ptr->node_range_x[0]),root->point.x);
            tmp_range_x[1] = max(max(left_son_ptr->node_range_x[1],right_son_ptr->node_range_x[1]),root->point.x);
            tmp_range_y[0] = min(min(left_son_ptr->node_range_y[0],right_son_ptr->node_range_y[0]),root->point.y);
            tmp_range_y[1] = max(max(left_son_ptr->node_range_y[1],right_son_ptr->node_range_y[1]),root->point.y);
            tmp_range_z[0] = min(min(left_son_ptr->node_range_z[0],right_son_ptr->node_range_z[0]),root->point.z);
            tmp_range_z[1] = max(max(left_son_ptr->node_range_z[1],right_son_ptr->node_range_z[1]),root->point.z);
        } else {
            if (!left_son_ptr->tree_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!right_son_ptr->tree_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);                
            }
            if (!root->point_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);                 
            }
        }
    } else if (left_son_ptr != nullptr){
        root->TreeSize = left_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + (root->point_deleted?1:0);
        root->down_del_num = left_son_ptr->down_del_num + (root->point_downsample_deleted?1:0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !root->point_deleted)){
            tmp_range_x[0] = min(left_son_ptr->node_range_x[0],root->point.x);
            tmp_range_x[1] = max(left_son_ptr->node_range_x[1],root->point.x);
            tmp_range_y[0] = min(left_son_ptr->node_range_y[0],root->point.y);
            tmp_range_y[1] = max(left_son_ptr->node_range_y[1],root->point.y); 
            tmp_range_z[0] = min(left_son_ptr->node_range_z[0],root->point.z);
            tmp_range_z[1] = max(left_son_ptr->node_range_z[1],root->point.z);  
        } else {
            if (!left_son_ptr->tree_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);                
            }
            if (!root->point_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);                 
            }            
        }

    } else if (right_son_ptr != nullptr){
        root->TreeSize = right_son_ptr->TreeSize + 1;
        root->invalid_point_num = right_son_ptr->invalid_point_num + (root->point_deleted? 1:0);
        root->down_del_num = right_son_ptr->down_del_num + (root->point_downsample_deleted? 1:0);        
        root->tree_downsample_deleted = right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!right_son_ptr->tree_deleted && !root->point_deleted)){
            tmp_range_x[0] = min(right_son_ptr->node_range_x[0],root->point.x);
            tmp_range_x[1] = max(right_son_ptr->node_range_x[1],root->point.x);
            tmp_range_y[0] = min(right_son_ptr->node_range_y[0],root->point.y);
            tmp_range_y[1] = max(right_son_ptr->node_range_y[1],root->point.y); 
            tmp_range_z[0] = min(right_son_ptr->node_range_z[0],root->point.z);
            tmp_range_z[1] = max(right_son_ptr->node_range_z[1],root->point.z); 
        } else {
            if (!right_son_ptr->tree_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);                
            }
            if (!root->point_deleted){
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);                 
            }            
        }
    } else {
        root->TreeSize = 1;
        root->invalid_point_num = (root->point_deleted? 1:0);
        root->down_del_num = (root->point_downsample_deleted? 1:0);
        root->tree_downsample_deleted = root->point_downsample_deleted;
        root->tree_deleted = root->point_deleted;
        tmp_range_x[0] = root->point.x;
        tmp_range_x[1] = root->point.x;        
        tmp_range_y[0] = root->point.y;
        tmp_range_y[1] = root->point.y; 
        tmp_range_z[0] = root->point.z;
        tmp_range_z[1] = root->point.z;                 
    }
    memcpy(root->node_range_x,tmp_range_x,sizeof(tmp_range_x));
    memcpy(root->node_range_y,tmp_range_y,sizeof(tmp_range_y));
    memcpy(root->node_range_z,tmp_range_z,sizeof(tmp_range_z));
    float x_L = (root->node_range_x[1] - root->node_range_x[0]) * 0.5;
    float y_L = (root->node_range_y[1] - root->node_range_y[0]) * 0.5;
    float z_L = (root->node_range_z[1] - root->node_range_z[0]) * 0.5;
    root->radius_sq = x_L*x_L + y_L * y_L + z_L * z_L;    
    if (left_son_ptr != nullptr) left_son_ptr -> father_ptr = root;
    if (right_son_ptr != nullptr) right_son_ptr -> father_ptr = root;
    if (root == Root_Node && root->TreeSize > 3){
        KD_TREE_NODE * son_ptr = root->left_son_ptr;
        if (son_ptr == nullptr) son_ptr = root->right_son_ptr;
        float tmp_bal = float(son_ptr->TreeSize) / (root->TreeSize-1);
        root->alpha_del = float(root->invalid_point_num)/ root->TreeSize;
        root->alpha_bal = (tmp_bal>=0.5-EPSS)?tmp_bal:1-tmp_bal;
    }   
    return;
}

template <typename PointType>
void KD_TREE<PointType>::checkCorrespondding(KD_TREE_NODE * root){
    if (root == nullptr) return;

    if (!root->point_deleted) {
        ++num_nodes;

        const PointType &p = root->point;
        VOXEL_LOC voxel_by_point;
        float cube_center[3];
        point2Voxel(p, voxel_by_point, cube_center);
    }

    checkCorrespondding(root->left_son_ptr);
    checkCorrespondding(root->right_son_ptr);
    return;
}

template <typename PointType>
void KD_TREE<PointType>::flatten(KD_TREE_NODE * root, PointVector &Storage, delete_point_storage_set storage_type){
    if (root == nullptr) return;
    Push_Down(root);
    if (!root->point_deleted) {
        Storage.push_back(root->point);
    }
    flatten(root->left_son_ptr, Storage, storage_type);
    flatten(root->right_son_ptr, Storage, storage_type);
    switch (storage_type)
    {
    case NOT_RECORD:
        break;
    case DELETE_POINTS_REC:
        if (root->point_deleted && !root->point_downsample_deleted) {
            Points_deleted.push_back(root->point);
        }       
        break;
    case MULTI_THREAD_REC:
        if (root->point_deleted  && !root->point_downsample_deleted) {
            Multithread_Points_deleted.push_back(root->point);
        }
        break;
    default:
        break;
    }     
    return;
}

template <typename PointType>
void KD_TREE<PointType>::delete_tree_nodes(KD_TREE_NODE ** root){ 
    if (*root == nullptr) return;
    Push_Down(*root);    
    delete_tree_nodes(&(*root)->left_son_ptr);
    delete_tree_nodes(&(*root)->right_son_ptr);
    
    pthread_mutex_destroy( &(*root)->push_down_mutex_lock);         
    delete *root;
    *root = nullptr;

    return;
}

template <typename PointType>
void KD_TREE<PointType>::node2Voxel(KD_TREE_NODE * root, VOXEL_LOC& voxel)
{
    point2Voxel(root->point, voxel);
}

template <typename PointType>
void KD_TREE<PointType>:: point2Voxel(const PointType& p, VOXEL_LOC& voxel) {
    //divide into voxel
    float loc_xyz[3];
    point2Voxel(p, loc_xyz);
    voxel = VOXEL_LOC((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
}

template <typename PointType>
void KD_TREE<PointType>:: point2Voxel(const PointType& p, float* loc_xyz, float* center_xyz) {
    point2Voxel(p, loc_xyz);
    voxelLoc2Center(loc_xyz, center_xyz);
}

template <typename PointType>
void KD_TREE<PointType>:: point2Voxel(const PointType& p, float* loc_xyz) {
    //divide into voxel
    loc_xyz[0] = floor(p.x / downsample_size);
    loc_xyz[1] = floor(p.y / downsample_size);
    loc_xyz[2] = floor(p.z / downsample_size);
}


template <typename PointType>
void KD_TREE<PointType>:: voxelLoc2Center(const float* loc_xyz, float* center_xyz)
{
    center_xyz[0] = loc_xyz[0] * downsample_size + half_voxel_size;
    center_xyz[1] = loc_xyz[1] * downsample_size + half_voxel_size;
    center_xyz[2] = loc_xyz[2] * downsample_size + half_voxel_size;
}

template <typename PointType>
void KD_TREE<PointType>:: point2Voxel(const PointType& p, VOXEL_LOC& voxel, float* center_cube) {
    //divide into voxel
    float loc_xyz[3];
    point2Voxel(p, loc_xyz);
    voxel = VOXEL_LOC((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    voxelLoc2Center(loc_xyz, center_cube);
}


template <typename PointType>
void KD_TREE<PointType>::single_connect(KD_TREE_NODE * root) {
    if (root == nullptr) return;
    if (!root->point_deleted)
    {
        const PointType& p = root->point;

        VOXEL_LOC voxel;
        point2Voxel(p, voxel);
        voxel2node[voxel] = root;
        auto iter = voxel2info.find(voxel);
        if (iter == voxel2info.end() || !iter->second) {
//            VoxelInfo voxel_new(p);
            voxel2info[voxel] = new VoxelInfo(p);
        }
    }
}

template <typename PointType>
void KD_TREE<PointType>::buildConnection(KD_TREE_NODE * root,  const bool & traverse) {
    if (root == nullptr) return;
//    Push_Down(root);
    if (!root->point_deleted)
    {
        const PointType& p = root->point;
        float loc_xyz[3];
        VOXEL_LOC voxel;
        point2Voxel(p, voxel);
        voxel2node[voxel] = root;

        auto iter = voxel2info.find(voxel);
        if (iter == voxel2info.end() || !iter->second) {
            voxel2info[voxel] = new VoxelInfo(p);
        }
    }
    if (!traverse)
        return;

    buildConnection(root->left_son_ptr);
    buildConnection(root->right_son_ptr);
}

template <typename PointType>
bool KD_TREE<PointType>::same_point(PointType a, PointType b){
    return (fabs(a.x-b.x) < EPSS && fabs(a.y-b.y) < EPSS && fabs(a.z-b.z) < EPSS );
}

template <typename PointType>
float KD_TREE<PointType>::calc_dist(PointType a, PointType b){
    float dist = 0.0f;
    dist = (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z);
    return dist;
}

template <typename PointType>
float KD_TREE<PointType>::calc_dist(const PointType& a, const Eigen::Vector3f& b){
    float dist = 0.0f;
    dist = (a.x-b(0))*(a.x-b(0)) + (a.y-b(1))*(a.y-b(1)) + (a.z-b(2))*(a.z-b(2));
    return dist;
}

template <typename PointType>
float KD_TREE<PointType>::calc_dist(const PointType& a, const float& x, const float& y, const float& z){
    float dist = 0.0f;
    dist = (a.x-x)*(a.x-x) + (a.y-y)*(a.y-y) + (a.z-z)*(a.z-z);
    return dist;
}

template <typename PointType>
float KD_TREE<PointType>::calc_dist(PointType a){
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

template <typename PointType>
float KD_TREE<PointType>::calc_box_dist(KD_TREE_NODE * node, PointType point){
    if (node == nullptr) return INFINITY;
    float min_dist = 0.0;
    if (point.x < node->node_range_x[0]) min_dist += (point.x - node->node_range_x[0])*(point.x - node->node_range_x[0]);
    if (point.x > node->node_range_x[1]) min_dist += (point.x - node->node_range_x[1])*(point.x - node->node_range_x[1]);
    if (point.y < node->node_range_y[0]) min_dist += (point.y - node->node_range_y[0])*(point.y - node->node_range_y[0]);
    if (point.y > node->node_range_y[1]) min_dist += (point.y - node->node_range_y[1])*(point.y - node->node_range_y[1]);
    if (point.z < node->node_range_z[0]) min_dist += (point.z - node->node_range_z[0])*(point.z - node->node_range_z[0]);
    if (point.z > node->node_range_z[1]) min_dist += (point.z - node->node_range_z[1])*(point.z - node->node_range_z[1]);
    return min_dist;
}

template <typename PointType> bool KD_TREE<PointType>::point_cmp_x(PointType a, PointType b) { return a.x < b.x;}
template <typename PointType> bool KD_TREE<PointType>::point_cmp_y(PointType a, PointType b) { return a.y < b.y;}
template <typename PointType> bool KD_TREE<PointType>::point_cmp_z(PointType a, PointType b) { return a.z < b.z;}

// manual queue
template <typename T>
void MANUAL_Q<T>::clear(){
    head = 0;
    tail = 0;
    counter = 0;
    is_empty = true;
    return;
}

template <typename T>
void MANUAL_Q<T>::pop(){
    if (counter == 0) return;
    head ++;
    head %= Q_LEN;
    counter --;
    if (counter == 0) is_empty = true;
    return;
}

template <typename T>
T MANUAL_Q<T>::front(){
    return q[head];
}

template <typename T>
T MANUAL_Q<T>::back(){
    return q[tail];
}

template <typename T>
void MANUAL_Q<T>::push(T op){
    q[tail] = op;
    counter ++;
    if (is_empty) is_empty = false;
    tail ++;
    tail %= Q_LEN;
}

template <typename T>
bool MANUAL_Q<T>::empty(){
    return is_empty;
}

template <typename T>
int MANUAL_Q<T>::size(){
    return counter;
}

//template class VoxelInfo<ikdTree_PointType>;
//template class KD_TREE<ikdTree_PointType>;
//template class KD_TREE<pcl::PointXYZ>;
//template class KD_TREE<pcl::PointXYZI>;
template class KD_TREE<pcl::PointXYZINormal>;