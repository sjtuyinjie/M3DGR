#include <stddef.h>
#include <vector>
#include <set>
#include "cloudMap.hpp"

struct SearchParams
{
     double radius = 0.5;
     double voxel_resolution = 0.;
     size_t map_id = 0;
     int voxel_neighborhood = 1;
};

struct ResolutionParam
{
     double resolution = 0.5;
     double min_distance_between_points = 0.1;
     int max_num_points = 40;
};

struct DistanceStrategy
{
     double distance_max = 30.; //< (m) Distance maximum for which the radius is maximum

     double radius_min = 0.1; //< (m) Minimum radius for points at distance 0m from the sensor

     double radius_max = 1.5; //< (m) Maximum radius for points at distance greater than `distance_max` from the sensor

     double exponent = 1.0; //< determines the search radius based on the distance to the sensor
};

struct mapOptions
{
     std::vector<ResolutionParam> resolutions = {
         ResolutionParam{0.2, 0.03, 30},
         ResolutionParam{0.5, 0.08, 40},
         ResolutionParam{1.5, 0.15, 40}};

     DistanceStrategy neig_options;
     double default_radius = 0.8;
};

class MultipleResolutionVoxelMap
{
public:
     MultipleResolutionVoxelMap() : MultipleResolutionVoxelMap(mapOptions()) {}
     ~MultipleResolutionVoxelMap() {}

     explicit MultipleResolutionVoxelMap(const mapOptions &options) : options_(options)
     {
          voxel_maps_.resize(options.resolutions.size());
     }

     void InsertPoint(const point3D &point);

     void InsertPointInVoxelMap(const point3D &point, size_t map_idx);

     void RemoveElementsFarFromLocation(const Eigen::Vector3d &location, double distance);

     void Reset(const mapOptions &options);

     size_t NumPoints() const;

     int NumVoxelMaps() const { return options_.resolutions.size(); }

     SearchParams SearchParamsFromRadiusSearch(double radius) const;

     SearchParams SearchParamsFromRadius(double radius) const;

     void RadiusSearchInPlace(const Eigen::Vector3d &query,
                              NeighborPoints &neighborhood,
                              double radius, int max_num_neighbors,
                              int threshold_voxel_capacity,
                              bool nearest_neighbors = true) const;

     /*!
      * @brief A Neighborhood strategy constructs neighborhood with radius which is adapted with the distance
      *
      * @note The radius is computed with the distance to the sensor, using the formula:
      *       $alpha= (min(radius, distance_{max}) / (distance_{max}))^{exponent}$
      *       $radius=(1.0 - alpha) * radius_{min} + alpha * radius_{max}$
      */
     double ComputeRadius(double distance_to_sensor) const;

private:
     mapOptions options_;
     struct VoxelHashMap
     {
          size_t num_points = 0;
          tsl::robin_map<voxel, voxelBlock> map;
     };

     using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

     struct __Comparator
     {
          bool operator()(const pair_distance_t &left, const pair_distance_t &right) const
          {
               return std::get<0>(left) < std::get<0>(right);
          }
     };
     typedef std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, __Comparator> priority_queue_t;

     std::vector<VoxelHashMap> voxel_maps_;
};
