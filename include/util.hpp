#include "types.hpp"

#include <pcl/filters/voxel_grid.h>

void downsample(cloud_ptr_t cloud_ptr, float leaf_size = 0.01f)
{
  pcl::VoxelGrid<point_t> sor;
  sor.setInputCloud (cloud_ptr);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (*cloud_ptr);
}
