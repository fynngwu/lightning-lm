#include "ui/publish_window.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace lightning::ui {

PublishWindow::PublishWindow(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

PublishWindow::~PublishWindow() { Quit(); }

bool PublishWindow::Init() {
    if (node_ == nullptr) {
        return false;
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    current_scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/current_scan", 10);
    history_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/global_map", 1);

    exit_.store(false);
    thread_ = std::thread(&PublishWindow::PublishWorker, this);
    return true;
}

void PublishWindow::Reset(const std::vector<Keyframe::Ptr>& keyframes) {
    (void)keyframes;

    std::lock_guard<std::mutex> lock(mtx_);
    tf_tasks_.clear();
    scan_tasks_.clear();
    global_map_cloud_->clear();
    scan_count_since_compact_ = 0;
}

void PublishWindow::UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud) { (void)cloud; }

void PublishWindow::UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud) { (void)cloud; }

void PublishWindow::UpdateNavState(const NavState& state) {
    if (node_ == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    latest_offset_R_lidar_ = state.offset_R_lidar_;
    latest_offset_t_lidar_ = state.offset_t_lidar_;

    if (!pub_tf_) {
        return;
    }

    TFTask task;
    task.map_rot_lidar = state.rot_ * state.offset_R_lidar_;
    task.map_pos_lidar = state.rot_ * state.offset_t_lidar_ + state.pos_;
    task.stamp = node_->get_clock()->now();

    tf_tasks_.clear();
    tf_tasks_.push_back(std::move(task));
    cv_.notify_one();
}

void PublishWindow::UpdateRecentPose(const SE3& pose) { (void)pose; }

void PublishWindow::UpdateScan(CloudPtr cloud, const SE3& pose) {
    if (cloud == nullptr || cloud->empty() || node_ == nullptr) {
        return;
    }

    auto scan_copy = std::make_shared<PointCloudType>();
    *scan_copy = *cloud;

    std::lock_guard<std::mutex> lock(mtx_);
    ScanTask task;
    task.scan = scan_copy;
    task.T_map_imu = pose;
    task.R_imu_lidar = latest_offset_R_lidar_;
    task.t_imu_lidar = latest_offset_t_lidar_;
    task.map_rot_lidar = pose.so3() * task.R_imu_lidar;
    task.map_pos_lidar = pose.so3() * task.t_imu_lidar + pose.translation();
    task.publish_tf = pub_tf_;
    task.publish_scan = pub_scan_;
    task.publish_map = pub_map_;
    task.stamp = node_->get_clock()->now();

    scan_tasks_.push_back(std::move(task));
    cv_.notify_one();
}

void PublishWindow::UpdateKF(std::shared_ptr<Keyframe> kf) { (void)kf; }

void PublishWindow::Quit() {
    exit_.store(true);
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool PublishWindow::ShouldQuit() { return false; }

void PublishWindow::SetTImuLidar(const SE3& T_imu_lidar) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_offset_R_lidar_ = T_imu_lidar.so3();
    latest_offset_t_lidar_ = T_imu_lidar.translation();
}

void PublishWindow::SetCurrentScanSize(int current_scan_size) {
    (void)current_scan_size;
}

void PublishWindow::SetPublishOptions(bool pub_tf, bool pub_scan, bool pub_map, double map_voxel_leaf_size) {
    std::lock_guard<std::mutex> lock(mtx_);
    pub_tf_ = pub_tf;
    pub_scan_ = pub_scan;
    pub_map_ = pub_map;
    if (map_voxel_leaf_size >= 0.0) {
        map_voxel_leaf_size_ = map_voxel_leaf_size;
    }
}

CloudPtr PublishWindow::TransformScanToMap(const PointCloudType& scan_lidar, const SE3& T_map_imu, const SO3& R_imu_lidar,
                                           const Vec3d& t_imu_lidar) const {
    auto scan_in_map = std::make_shared<PointCloudType>();
    scan_in_map->reserve(scan_lidar.size());

    const SO3 map_rot_lidar = T_map_imu.so3() * R_imu_lidar;
    const Vec3d map_pos_lidar = T_map_imu.so3() * t_imu_lidar + T_map_imu.translation();

    for (const auto& pt_lidar : scan_lidar.points) {
        PointType pt_map = pt_lidar;
        const Vec3d p_lidar(static_cast<double>(pt_lidar.x), static_cast<double>(pt_lidar.y),
                            static_cast<double>(pt_lidar.z));
        const Vec3d p_map = map_rot_lidar * p_lidar + map_pos_lidar;
        pt_map.x = static_cast<float>(p_map.x());
        pt_map.y = static_cast<float>(p_map.y());
        pt_map.z = static_cast<float>(p_map.z());
        scan_in_map->push_back(pt_map);
    }

    scan_in_map->width = static_cast<uint32_t>(scan_in_map->size());
    scan_in_map->height = 1;
    scan_in_map->is_dense = scan_lidar.is_dense;
    return scan_in_map;
}

void PublishWindow::DownsampleCloudInplace(const CloudPtr& cloud, double voxel_leaf_size) const {
    if (cloud == nullptr || cloud->empty() || voxel_leaf_size <= 1e-6) {
        return;
    }

    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(static_cast<float>(voxel_leaf_size), static_cast<float>(voxel_leaf_size),
                             static_cast<float>(voxel_leaf_size));
    voxel_filter.setInputCloud(cloud);
    PointCloudType filtered;
    voxel_filter.filter(filtered);
    *cloud = std::move(filtered);
}

void PublishWindow::PublishWorker() {
    while (true) {
        TFTask tf_task;
        ScanTask scan_task;
        bool has_tf_task = false;
        bool has_scan_task = false;

        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [this]() { return exit_.load() || !scan_tasks_.empty() || !tf_tasks_.empty(); });

            if (exit_.load() && scan_tasks_.empty() && tf_tasks_.empty()) {
                break;
            }

            if (!scan_tasks_.empty()) {
                scan_task = std::move(scan_tasks_.front());
                scan_tasks_.pop_front();
                has_scan_task = true;
            } else if (!tf_tasks_.empty()) {
                tf_task = std::move(tf_tasks_.front());
                tf_tasks_.pop_front();
                has_tf_task = true;
            }
        }

        if (has_tf_task && tf_broadcaster_ != nullptr) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = tf_task.stamp;
            tf_msg.header.frame_id = "map";
            tf_msg.child_frame_id = "lidar";

            tf_msg.transform.translation.x = tf_task.map_pos_lidar.x();
            tf_msg.transform.translation.y = tf_task.map_pos_lidar.y();
            tf_msg.transform.translation.z = tf_task.map_pos_lidar.z();
            tf_msg.transform.rotation.x = tf_task.map_rot_lidar.unit_quaternion().x();
            tf_msg.transform.rotation.y = tf_task.map_rot_lidar.unit_quaternion().y();
            tf_msg.transform.rotation.z = tf_task.map_rot_lidar.unit_quaternion().z();
            tf_msg.transform.rotation.w = tf_task.map_rot_lidar.unit_quaternion().w();
            tf_broadcaster_->sendTransform(tf_msg);
        }

        if (has_scan_task) {
            if (scan_task.publish_tf && tf_broadcaster_ != nullptr) {
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = scan_task.stamp;
                tf_msg.header.frame_id = "map";
                tf_msg.child_frame_id = "lidar";

                tf_msg.transform.translation.x = scan_task.map_pos_lidar.x();
                tf_msg.transform.translation.y = scan_task.map_pos_lidar.y();
                tf_msg.transform.translation.z = scan_task.map_pos_lidar.z();
                tf_msg.transform.rotation.x = scan_task.map_rot_lidar.unit_quaternion().x();
                tf_msg.transform.rotation.y = scan_task.map_rot_lidar.unit_quaternion().y();
                tf_msg.transform.rotation.z = scan_task.map_rot_lidar.unit_quaternion().z();
                tf_msg.transform.rotation.w = scan_task.map_rot_lidar.unit_quaternion().w();
                tf_broadcaster_->sendTransform(tf_msg);
            }

            if (scan_task.publish_scan && current_scan_pub_ != nullptr) {
                sensor_msgs::msg::PointCloud2 scan_msg;
                pcl::toROSMsg(*scan_task.scan, scan_msg);
                scan_msg.header.stamp = scan_task.stamp;
                scan_msg.header.frame_id = "lidar";
                current_scan_pub_->publish(scan_msg);
            }

            if (scan_task.publish_map && history_map_pub_ != nullptr) {
                auto scan_in_map =
                    TransformScanToMap(*scan_task.scan, scan_task.T_map_imu, scan_task.R_imu_lidar, scan_task.t_imu_lidar);

                double voxel_leaf_size = 0.0;
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    voxel_leaf_size = map_voxel_leaf_size_;
                }
                DownsampleCloudInplace(scan_in_map, voxel_leaf_size);

                CloudPtr map_snapshot;
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    if (scan_in_map != nullptr && !scan_in_map->empty()) {
                        *global_map_cloud_ += *scan_in_map;
                        ++scan_count_since_compact_;

                        if (scan_count_since_compact_ >= global_map_compact_every_n_) {
                            DownsampleCloudInplace(global_map_cloud_, voxel_leaf_size);
                            scan_count_since_compact_ = 0;
                        }
                    }

                    map_snapshot = std::make_shared<PointCloudType>();
                    *map_snapshot = *global_map_cloud_;
                }

                sensor_msgs::msg::PointCloud2 map_msg;
                pcl::toROSMsg(*map_snapshot, map_msg);
                map_msg.header.stamp = scan_task.stamp;
                map_msg.header.frame_id = "map";
                history_map_pub_->publish(map_msg);
            }
        }
    }
}

}  // namespace lightning::ui
