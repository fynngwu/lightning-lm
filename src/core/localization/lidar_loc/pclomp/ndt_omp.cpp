#include "ndt_omp.h"
#include "ndt_omp_impl.hpp"

#include "common/point_def.h"

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
template class pclomp::NormalDistributionsTransform<PointXYZIT, PointXYZIT>;
