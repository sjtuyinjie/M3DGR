#include "rgbMapTracker.h"

rgbMapTracker::rgbMapTracker()
{
	std::vector<point3D> v_point_temp;
	state *p_state_ = new state();
	p_cloud_frame = new cloudFrame(v_point_temp, p_state_);

	minimum_depth_for_projection = 0.1;
    maximum_depth_for_projection = 200;

	recent_visited_voxel_activated_time = 1.0;

	number_of_new_visited_voxel = 0;

	updated_frame_index = -1;

	in_appending_points = false;

	points_rgb_vec_for_projection = nullptr;

    mutex_rgb_points_vec = std::make_shared<std::mutex>();
    mutex_frame_index = std::make_shared<std::mutex>();
}

void rgbMapTracker::refreshPointsForProjection(voxelHashMap &map)
{
	cloudFrame *p_frame = p_cloud_frame;

	if (p_frame->image_cols == 0 || p_frame->image_rows == 0) return;

	if (p_frame->frame_id == updated_frame_index) return;

	std::vector<rgbPoint*> *points_rgb_vec_for_projection_temp = new std::vector<rgbPoint*>();

	selectPointsForProjection(map, p_frame, points_rgb_vec_for_projection_temp, nullptr, 10.0, 1);

	points_rgb_vec_for_projection = points_rgb_vec_for_projection_temp;

    mutex_frame_index->lock();
	updated_frame_index = p_frame->frame_id;
    mutex_frame_index->unlock();
}

void rgbMapTracker::selectPointsForProjection(voxelHashMap &map, cloudFrame *p_frame, std::vector<rgbPoint*> *pc_out_vec, 
	std::vector<cv::Point2f> *pc_2d_out_vec, double minimum_dis, int skip_step, bool use_all_points)
{
	if (pc_out_vec != nullptr)
    {
        pc_out_vec->clear();
    }

    if (pc_2d_out_vec != nullptr)
    {
        pc_2d_out_vec->clear();
    }

    Hash_map_2d<int, int> mask_index;
    Hash_map_2d<int, float> mask_depth;

    std::map<int, cv::Point2f> map_idx_draw_center;
    std::map<int, cv::Point2f> map_idx_draw_center_raw_pose;

    int u, v;
    double u_f, v_f;

    int acc = 0;
    int blk_rej = 0;

    std::vector<rgbPoint*> points_for_projection;

    std::vector<voxelId> boxes_recent_hitted = voxels_recent_visited;

    if ((!use_all_points) && boxes_recent_hitted.size())
    {
    	for(std::vector<voxelId>::iterator it = boxes_recent_hitted.begin(); it != boxes_recent_hitted.end(); it++)
        {
            if (map[voxel((*it).kx, (*it).ky, (*it).kz)].NumPoints() > 0)
            {
                points_for_projection.push_back(&(map[voxel((*it).kx, (*it).ky, (*it).kz)].points.back()));
            }
        }
    }
    else
    {
        mutex_rgb_points_vec->lock();
        points_for_projection = rgb_points_vec;
        mutex_rgb_points_vec->unlock();
    }

    int point_size = points_for_projection.size();

    for (int point_index = 0; point_index < point_size; point_index += skip_step)
    {
        Eigen::Vector3d point_world = points_for_projection[point_index]->getPosition();

        double depth = (point_world - p_frame->p_state->t_world_camera).norm();

        if (depth > maximum_depth_for_projection)
        {
            continue;
        }

        if (depth < minimum_depth_for_projection)
        {
            continue;
        }

        bool res = p_frame->project3dPointInThisImage(point_world, u_f, v_f, nullptr, 1.0);

        if (res == false)
        {
            continue;
        }

        u = std::round(u_f / minimum_dis) * minimum_dis;
        v = std::round(v_f / minimum_dis) * minimum_dis;

        if ((!mask_depth.if_exist(u, v)) || mask_depth.m_map_2d_hash_map[u][v] > depth)
        {
            acc++;

            if (mask_index.if_exist(u, v))
            {
                int old_idx = mask_index.m_map_2d_hash_map[u][v];

                blk_rej++;

                map_idx_draw_center.erase(map_idx_draw_center.find(old_idx));
                map_idx_draw_center_raw_pose.erase(map_idx_draw_center_raw_pose.find(old_idx));
            }

            mask_index.m_map_2d_hash_map[u][v] = (int)point_index;
            mask_depth.m_map_2d_hash_map[u][v] = (float)depth;

            map_idx_draw_center[point_index] = cv::Point2f(v, u);
            map_idx_draw_center_raw_pose[point_index] = cv::Point2f(u_f, v_f);
        }
    }

    if (pc_out_vec != nullptr)
    {
        for (auto it = map_idx_draw_center.begin(); it != map_idx_draw_center.end(); it++)
            pc_out_vec->push_back(points_for_projection[it->first]);
    }

    if (pc_2d_out_vec != nullptr)
    {
        for (auto it = map_idx_draw_center.begin(); it != map_idx_draw_center.end(); it++)
            pc_2d_out_vec->push_back(map_idx_draw_center_raw_pose[it->first]);
    }
}

void rgbMapTracker::updatePoseForProjection(cloudFrame *p_frame, double fov_margin)
{
	p_cloud_frame->p_state->fx = p_frame->p_state->fx;
	p_cloud_frame->p_state->fy = p_frame->p_state->fy;
	p_cloud_frame->p_state->cx = p_frame->p_state->cx;
	p_cloud_frame->p_state->cy = p_frame->p_state->cy;

	p_cloud_frame->image_cols = p_frame->image_cols;
	p_cloud_frame->image_rows = p_frame->image_rows;

	p_cloud_frame->p_state->fov_margin = fov_margin;
	p_cloud_frame->frame_id = p_frame->frame_id;

	p_cloud_frame->p_state->q_world_camera = p_frame->p_state->q_world_camera;
	p_cloud_frame->p_state->t_world_camera = p_frame->p_state->t_world_camera;

	p_cloud_frame->rgb_image = p_frame->rgb_image;
	p_cloud_frame->gray_image = p_frame->gray_image;

    p_cloud_frame->refreshPoseForProjection();
}

const double image_obs_cov = 15;
const double process_noise_sigma = 0.1;

std::atomic<long> render_point_count;

void rgbMapTracker::threadRenderPointsInVoxel(voxelHashMap &map, const int &voxel_start, const int &voxel_end, cloudFrame *p_frame, 
	const std::vector<voxelId> *voxels_for_render, const double obs_time)
{
	Eigen::Vector3d point_world;
	Eigen::Vector3d point_color;

	double u, v;
	double point_camera_norm;

	for (int voxel_index = voxel_start; voxel_index < voxel_end; voxel_index++)
	{
        voxelBlock &voxel_block = map[voxel((*voxels_for_render)[voxel_index].kx, (*voxels_for_render)[voxel_index].ky, (*voxels_for_render)[voxel_index].kz)];

        for (int point_index = 0; point_index < voxel_block.NumPoints(); point_index++)
        {
        	auto &point = voxel_block.points[point_index];

        	point_world = point.getPosition();

        	if (p_frame->project3dPointInThisImage(point_world, u, v, nullptr, 1.0) == false) continue;

        	point_camera_norm = (point_world - p_frame->p_state->t_world_camera).norm();

        	point_color = p_frame->getRgb(u, v, 0);

            mutex_rgb_points_vec->lock();
        	if (voxel_block.points[point_index].updateRgb(point_color, point_camera_norm, 
        		Eigen::Vector3d(image_obs_cov, image_obs_cov, image_obs_cov), obs_time))
        	{
        		render_point_count++;
        	}
            mutex_rgb_points_vec->unlock();
        }
	}
}

std::vector<voxelId> g_voxel_for_render;

void rgbMapTracker::renderPointsInRecentVoxel(voxelHashMap &map, cloudFrame *p_frame, std::vector<voxelId> *voxels_for_render, const double &obs_time)
{
	g_voxel_for_render.clear();
    std::vector<voxelId>().swap(g_voxel_for_render);

	for (std::vector<voxelId>::iterator it = (*voxels_for_render).begin(); it != (*voxels_for_render).end(); it++)
	{
		g_voxel_for_render.push_back(*it);
	}

	std::vector<std::future<double>> results;

	int number_of_voxels = g_voxel_for_render.size();

	render_point_count = 0;

	cv::parallel_for_(cv::Range(0, number_of_voxels), [&](const cv::Range &r)
					{ threadRenderPointsInVoxel(map, r.start, r.end, p_frame, &g_voxel_for_render, obs_time); });
}