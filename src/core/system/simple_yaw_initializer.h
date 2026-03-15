#pragma once

#include <mutex>
#include <string>

#include <pcl/registration/icp.h>

#include "common/constant.h"
#include "common/eigen_types.h"
#include "common/point_def.h"
#include "core/lightning_math.hpp"
#include "core/localization/lidar_loc/pclomp/ndt_omp_impl.hpp"
#include "core/maps/tiled_map.h"

namespace lightning {

class SimpleYawInitializer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Options {
        TiledMap::Options map_options_;
        float min_init_confidence_ = 1.0f;
        int yaw_search_step_ = 36;
        double yaw_search_range_deg_ = 180.0;
        bool enable_icp_adjust_ = false;
    };

    bool Init(const std::string& yaml_path, const std::string& map_path);

    bool InitOnce(CloudPtr scan_undist, const SE3& init_pose, SE3& result_pose, double& score, CloudPtr& loaded_local_map);

   private:
    using NDTType = pclomp::NormalDistributionsTransform<PointType, PointType>;
    using ICPType = pcl::IterativeClosestPoint<PointType, PointType>;
    using UL = std::unique_lock<std::mutex>;

    bool BuildTargets();
    bool Localize(SE3& pose, double& score, CloudPtr input, CloudPtr output, bool rough);
    bool YawSearch(SE3& pose, double& score, CloudPtr input, CloudPtr output);

    Options options_;
    std::mutex match_mutex_;
    std::shared_ptr<TiledMap> map_ = nullptr;
    NDTType::Ptr ndt_rough_ = nullptr;
    NDTType::Ptr ndt_fine_ = nullptr;
    ICPType::Ptr icp_ = nullptr;
};

}  // namespace lightning
