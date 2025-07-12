#include "map.hpp"

void MultipleResolutionVoxelMap::InsertPoint(const point3D &point)
{
     for (auto map_idx(0); map_idx < options_.resolutions.size(); map_idx++)
          InsertPointInVoxelMap(point, map_idx);
}

void MultipleResolutionVoxelMap::
    InsertPointInVoxelMap(const point3D &point, size_t map_index)
{
     const auto &res_param = options_.resolutions[map_index];
     auto &hash_map_ = voxel_maps_[map_index];
     voxel vo = voxel::coordinates(point.point, res_param.resolution);

     if (hash_map_.map.find(vo) == hash_map_.map.end())
     {
          hash_map_.map[vo].points.reserve(res_param.max_num_points);
          hash_map_.map[vo].points.push_back(point.point);
          hash_map_.num_points++;
          return;
     }
     auto &voxel_block = hash_map_.map[vo];
     if (voxel_block.points.size() < res_param.max_num_points)
     {
          double sq_dist_min_to_points = std::numeric_limits<double>::max();
          // Insert a point only if it is greader than the min distance between points
          for (int i(0); i < voxel_block.points.size(); ++i)
          {
               auto &_point = voxel_block.points[i];
               double sq_dist = (_point.cast<double>() - point.point).squaredNorm();
               if (sq_dist < sq_dist_min_to_points)
               {
                    sq_dist_min_to_points = sq_dist;
               }
          }
          if (sq_dist_min_to_points > (res_param.min_distance_between_points * res_param.min_distance_between_points))
          {
               voxel_block.points.push_back(point.point);
               hash_map_.num_points++;
               return;
          }
     }
}

void MultipleResolutionVoxelMap::RemoveElementsFarFromLocation(const Eigen::Vector3d &location, double distance)
{ // Iterate over all voxels and suppress the voxels to remove
     for (auto map_idx = 0; map_idx < voxel_maps_.size(); map_idx++)
     {
          std::set<voxel> voxels_to_remove;
          auto &map = voxel_maps_[map_idx].map;
          for (auto &pair : voxel_maps_[map_idx].map)
          {
               if (pair.second.points.empty())
                    voxels_to_remove.insert(pair.first);
               if ((pair.second.points.front() - location).norm() > distance)
                    voxels_to_remove.insert(pair.first);
          }

          for (auto &vox : voxels_to_remove)
          {
               voxel_maps_[map_idx].num_points -= map[vox].points.size();
               map.erase(vox);
          }
     }
}

void MultipleResolutionVoxelMap::Reset(const mapOptions &options)
{
     options_ = options;
     voxel_maps_.resize(0);
     voxel_maps_.resize(options.resolutions.size());
}

size_t MultipleResolutionVoxelMap::NumPoints() const
{
     return voxel_maps_.front().num_points;
}

SearchParams MultipleResolutionVoxelMap::SearchParamsFromRadiusSearch(double radius) const
{
     SearchParams params;
     auto it = std::lower_bound(options_.resolutions.begin(),
                                options_.resolutions.end(), radius,
                                [](const ResolutionParam &lhs, double radius)
                                {
                                     return lhs.resolution <= radius;
                                });
     auto idx = std::max(int(0),
                         int(std::distance(options_.resolutions.begin(), it)) - 1);
     params.radius = radius;
     params.map_id = idx;
     double resolution = options_.resolutions[idx].resolution;
     params.voxel_resolution = resolution;
     params.voxel_neighborhood = 1; // std::ceil(radius / resolution);

     std::cout << "map_id: " << params.map_id
               << ", radius: " << radius
               << ", resolution: " << resolution
               << ", voxel_neighborhood: " << params.voxel_neighborhood << std::endl;

     return params;
}

SearchParams MultipleResolutionVoxelMap::SearchParamsFromRadius(double radius) const
{
     SearchParams params;
     if (radius < 10)
          params.map_id = 0;
     else if (radius < 40)
          params.map_id = 1;
     else
          params.map_id = 2;

     params.radius = radius;
     params.voxel_resolution = options_.resolutions[params.map_id].resolution;
     params.voxel_neighborhood = 1;

     return params;
}

double MultipleResolutionVoxelMap::ComputeRadius(double distance_to_sensor) const
{
     double alpha = std::pow(std::min(std::abs(distance_to_sensor),
                                      options_.neig_options.distance_max) /
                                 options_.neig_options.radius_max,
                             options_.neig_options.exponent);
     return alpha * options_.neig_options.radius_max + (1 - alpha) * options_.neig_options.radius_min;
}

void MultipleResolutionVoxelMap::RadiusSearchInPlace(const Eigen::Vector3d &query,
                                                     NeighborPoints &neighborhood,
                                                     double radius, int max_num_neighbors,
                                                     int threshold_voxel_capacity,
                                                     bool nearest_neighbors) const
{
     neighborhood.resize(0);
     neighborhood.reserve(max_num_neighbors);
     // const SearchParams params = SearchParamsFromRadiusSearch(radius);
     const SearchParams params = SearchParamsFromRadius(radius);

     const auto &hash_map_ = voxel_maps_[params.map_id].map;
     const double voxel_size = params.voxel_resolution;
     const int nb_voxels_visited = params.voxel_neighborhood;
     const double max_neighborhood_radius = params.radius;
     voxel vox = voxel::coordinates(query, voxel_size);
     int kx = vox.x;
     int ky = vox.y;
     int kz = vox.z;

     priority_queue_t priority_queue;
     size_t num_points_skipped = 0;
     for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx)
     {
          for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy)
          {
               for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz)
               {
                    vox.x = kxx;
                    vox.y = kyy;
                    vox.z = kzz;

                    auto search = hash_map_.find(vox);
                    if (search != hash_map_.end())
                    {
                         const auto &voxel_block = search.value();
                         if (voxel_block.NumPoints() < threshold_voxel_capacity)
                              continue;
                         for (int i(0); i < voxel_block.NumPoints(); ++i)
                         {
                              auto &neighbor = voxel_block.points[i];
                              double distance = (neighbor - query).norm();
                              if (priority_queue.size() == max_num_neighbors)
                              {
                                   if (distance < std::get<0>(priority_queue.top()))
                                   {
                                        priority_queue.pop();
                                        priority_queue.emplace(distance, neighbor, vox);
                                   }
                              }
                              else
                                   priority_queue.emplace(distance, neighbor, vox);
                         }
                    }
               }
          }
     }
     neighborhood.resize(0);
     neighborhood.reserve(priority_queue.size());
     while (!priority_queue.empty())
     {
          neighborhood.push_back(std::get<1>(priority_queue.top()));
          priority_queue.pop();
     }

     return;
}