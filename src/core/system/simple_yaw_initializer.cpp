#include "core/system/simple_yaw_initializer.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pcl/filters/voxel_grid.h>

#include "glog/logging.h"
#include "io/yaml_io.h"

namespace lightning {

bool SimpleYawInitializer::Init(const std::string& yaml_path, const std::string& map_path) {
    YAML_IO yaml(yaml_path);
    options_.map_options_.enable_dynamic_polygon_ = yaml.GetValue<bool>("maps", "with_dyn_area");
    options_.map_options_.max_pts_in_dyn_chunk_ = yaml.GetValue<int>("maps", "max_pts_dyn_chunk");
    options_.map_options_.load_map_size_ = yaml.GetValue<int>("maps", "load_map_size");
    options_.map_options_.unload_map_size_ = yaml.GetValue<int>("maps", "unload_map_size");
    options_.map_options_.delete_when_unload_ = yaml.GetValue<bool>("maps", "delete_when_unload");
    options_.map_options_.load_dyn_cloud_ = yaml.GetValue<bool>("maps", "load_dyn_cloud");
    options_.map_options_.save_dyn_when_quit_ = yaml.GetValue<bool>("maps", "save_dyn_when_quit");
    options_.map_options_.save_dyn_when_unload_ = yaml.GetValue<bool>("maps", "save_dyn_when_unload");
    options_.map_options_.map_path_ = map_path;

    std::string map_policy = yaml.GetValue<std::string>("maps", "dyn_cloud_policy");
    if (map_policy == "short") {
        options_.map_options_.policy_ = TiledMap::DynamicCloudPolicy::SHORT;
    } else if (map_policy == "long") {
        options_.map_options_.policy_ = TiledMap::DynamicCloudPolicy::LONG;
    } else {
        options_.map_options_.policy_ = TiledMap::DynamicCloudPolicy::PERSISTENT;
    }

    options_.min_init_confidence_ = yaml.GetValue<float>("lidar_loc", "min_init_confidence");
    options_.yaw_search_step_ = yaml.GetValue<int>("lidar_loc", "grid_search_angle_step");
    options_.yaw_search_range_deg_ = yaml.GetValue<double>("lidar_loc", "grid_search_angle_range");
    options_.enable_icp_adjust_ = yaml.GetValue<bool>("lidar_loc", "enable_icp_adjust");

    map_ = std::make_shared<TiledMap>(options_.map_options_);
    if (!map_->LoadMapIndex()) {
        LOG(ERROR) << "failed to load tiled map index from " << map_path;
        return false;
    }

    return true;
}

bool SimpleYawInitializer::BuildTargets() {
    ndt_fine_.reset(new NDTType());
    ndt_fine_->setResolution(1.0);
    ndt_fine_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt_fine_->setStepSize(0.1);
    ndt_fine_->setMaximumIterations(4);
    ndt_fine_->setNumThreads(4);
    map_->SetNewTargetForNDT(ndt_fine_);

    ndt_rough_.reset(new NDTType());
    ndt_rough_->setResolution(5.0);
    ndt_rough_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt_rough_->setStepSize(0.1);
    ndt_rough_->setMaximumIterations(4);
    ndt_rough_->setNumThreads(4);
    map_->SetNewTargetForNDT(ndt_rough_);

    if (options_.enable_icp_adjust_) {
        pcl::VoxelGrid<PointType> voxel;
        CloudPtr map_cloud(new PointCloudType);
        voxel.setLeafSize(0.5f, 0.5f, 0.5f);
        voxel.setInputCloud(map_->GetAllMap());
        voxel.filter(*map_cloud);

        icp_.reset(new ICPType());
        icp_->setInputTarget(map_cloud);
        icp_->setMaximumIterations(4);
        icp_->setTransformationEpsilon(0.01);
    } else {
        icp_.reset();
    }

    return ndt_fine_->getInputTarget() != nullptr && ndt_rough_->getInputTarget() != nullptr;
}

bool SimpleYawInitializer::InitOnce(CloudPtr scan_undist, const SE3& init_pose, SE3& result_pose, double& score,
                                    CloudPtr& loaded_local_map) {
    if (scan_undist == nullptr || scan_undist->empty()) {
        return false;
    }

    map_->LoadOnPose(init_pose);
    if (!BuildTargets()) {
        LOG(WARNING) << "online init target map is empty";
        return false;
    }

    result_pose = init_pose;
    CloudPtr output_cloud(new PointCloudType);
    if (!YawSearch(result_pose, score, scan_undist, output_cloud)) {
        return false;
    }

    loaded_local_map = map_->GetAllMap();
    return loaded_local_map != nullptr && !loaded_local_map->empty();
}

bool SimpleYawInitializer::YawSearch(SE3& pose, double& score, CloudPtr input, CloudPtr output) {
    PoseRPYD rpyxyz = math::SE3ToRollPitchYaw(pose);
    const double init_yaw = rpyxyz.yaw;
    const int step = std::max(1, options_.yaw_search_step_);
    const double radius = options_.yaw_search_range_deg_ * constant::kDEG2RAD;
    const double yaw_step = step > 1 ? (2.0 * radius / static_cast<double>(step)) : 0.0;

    score = 0.0;
    double best_score = -std::numeric_limits<double>::infinity();
    SE3 best_pose = pose;

    for (int i = 0; i < step; ++i) {
        rpyxyz.yaw = step > 1 ? (init_yaw + static_cast<double>(i) * yaw_step - radius) : init_yaw;
        SE3 pose_esti = math::XYZRPYToSE3(rpyxyz);
        double candidate_score = 0.0;
        Localize(pose_esti, candidate_score, input, output, true);
        if (candidate_score > best_score) {
            best_score = candidate_score;
            best_pose = pose_esti;
        }
    }

    pose = best_pose;
    score = best_score;

    if (score > options_.min_init_confidence_) {
        Localize(pose, score, input, output, false);
    }

    return score > options_.min_init_confidence_;
}

bool SimpleYawInitializer::Localize(SE3& pose, double& score, CloudPtr input, CloudPtr output, bool rough) {
    if (input == nullptr || input->empty()) {
        return false;
    }

    NDTType::Ptr ndt = rough ? ndt_rough_ : ndt_fine_;
    if (ndt == nullptr || ndt->getInputTarget() == nullptr) {
        return false;
    }

    UL lock(match_mutex_);

    ndt->setInputSource(input);
    ndt->align(*output, pose.matrix().cast<float>());

    Eigen::Matrix4f trans = ndt->getFinalTransformation();
    score = ndt->getTransformationProbability();

    if (options_.enable_icp_adjust_ && !rough && icp_ != nullptr) {
        pcl::VoxelGrid<PointType> voxel;
        CloudPtr input_voxel(new PointCloudType);
        voxel.setLeafSize(0.2f, 0.2f, 0.2f);
        voxel.setInputCloud(input);
        voxel.filter(*input_voxel);

        icp_->setInputSource(input_voxel);
        icp_->align(*output, trans);

        const Eigen::Matrix4f adjust_trans = icp_->getFinalTransformation();
        const Eigen::Matrix3f rotation_diff = trans.block<3, 3>(0, 0).transpose() * adjust_trans.block<3, 3>(0, 0);
        const Eigen::AngleAxisf angle_axis(rotation_diff);
        const float delta_angle = angle_axis.angle();
        const float delta_trans = (trans.block<3, 1>(0, 3) - adjust_trans.block<3, 1>(0, 3)).norm();

        if (icp_->hasConverged() && std::fabs(delta_trans) <= 0.05f && std::fabs(delta_angle) <= 0.05f) {
            trans = adjust_trans;
        }
    }

    Quatd q(trans.block<3, 3>(0, 0).cast<double>());
    q.normalize();
    pose = SE3(q, trans.block<3, 1>(0, 3).cast<double>());
    return true;
}

}  // namespace lightning
