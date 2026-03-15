#include "voxel_grid_covariance_omp.h"
#include "voxel_grid_covariance_omp_impl.hpp"

#include "common/point_def.h"

template class pclomp::VoxelGridCovariance<pcl::PointXYZ>;
template class pclomp::VoxelGridCovariance<pcl::PointXYZI>;
template class pclomp::VoxelGridCovariance<PointXYZIT>;
