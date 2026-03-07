#include "ui/publish_window.h"

#include <algorithm>
#include <chrono>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace lightning::ui {

PublishWindow::PublishWindow(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

PublishWindow::~PublishWindow() { Quit(); }

bool PublishWindow::Init() {
    if (node_ == nullptr) {
        return false;
    }

    if (pub_rate_hz_ <= 0.0) {
        pub_rate_hz_ = 20.0;
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    current_scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/current_scan", 10);
    history_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/global_map", 1);

    exit_.store(false);
    thread_ = std::thread(&PublishWindow::PublishLoop, this);
    return true;
}

void PublishWindow::Reset(const std::vector<Keyframe::Ptr>& keyframes) {
    (void)keyframes;
    std::lock_guard<std::mutex> lock(mtx_);
    history_scans_.clear();
    merged_history_cloud_->clear();
    history_need_rebuild_ = false;
}

void PublishWindow::UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud) { (void)cloud; }

void PublishWindow::UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud) { (void)cloud; }

void PublishWindow::UpdateNavState(const NavState& state) {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_state_ = state;
    latest_offset_R_lidar_ = state.offset_R_lidar_;
    latest_offset_t_lidar_ = state.offset_t_lidar_;
    has_latest_state_ = true;
}

void PublishWindow::UpdateRecentPose(const SE3& pose) { (void)pose; }

void PublishWindow::UpdateScan(CloudPtr cloud, const SE3& pose) {
    if (cloud == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    *latest_scan_ = *cloud;
    has_latest_scan_ = !latest_scan_->empty();
    latest_scan_pose_ = pose;
    has_latest_scan_pose_ = true;
    latest_scan_offset_R_lidar_ = latest_offset_R_lidar_;
    latest_scan_offset_t_lidar_ = latest_offset_t_lidar_;
    ++latest_scan_seq_;
}

void PublishWindow::UpdateKF(std::shared_ptr<Keyframe> kf) { (void)kf; }

void PublishWindow::Quit() {
    exit_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool PublishWindow::ShouldQuit() { return false; }

void PublishWindow::SetTImuLidar(const SE3& T_imu_lidar) {
    std::lock_guard<std::mutex> lock(mtx_);
    T_imu_lidar_ = T_imu_lidar;
}

void PublishWindow::SetCurrentScanSize(int current_scan_size) {
    if (current_scan_size <= 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    history_scan_max_size_ = static_cast<size_t>(current_scan_size);
    while (history_scans_.size() > history_scan_max_size_) {
        history_scans_.pop_front();
    }
    history_need_rebuild_ = true;
}

void PublishWindow::SetPublishOptions(bool pub_tf, bool pub_scan, bool pub_map, double pub_rate_hz) {
    std::lock_guard<std::mutex> lock(mtx_);
    pub_tf_ = pub_tf;
    pub_scan_ = pub_scan;
    pub_map_ = pub_map;
    if (pub_rate_hz > 0.0) {
        pub_rate_hz_ = pub_rate_hz;
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

void PublishWindow::AppendHistoryScan(const CloudPtr& scan_in_map) {
    if (scan_in_map == nullptr || scan_in_map->empty()) {
        return;
    }

    history_scans_.push_back(scan_in_map);
    *merged_history_cloud_ += *scan_in_map;

    if (history_scans_.size() > history_scan_max_size_) {
        history_scans_.pop_front();
        history_need_rebuild_ = true;
    }

    if (history_need_rebuild_) {
        merged_history_cloud_->clear();
        for (const auto& cloud : history_scans_) {
            if (cloud == nullptr || cloud->empty()) {
                continue;
            }
            *merged_history_cloud_ += *cloud;
        }
        history_need_rebuild_ = false;
    }
}

void PublishWindow::PublishLoop() {
    while (!exit_.load()) {
        double rate_hz = 20.0;
        bool do_pub_tf = false;
        bool do_pub_scan = false;
        bool do_pub_map = false;

        CloudPtr scan_snapshot;
        CloudPtr history_snapshot;
        bool has_tf_pose = false;
        SO3 tf_map_rot_lidar;
        Vec3d tf_map_pos_lidar = Vec3d::Zero();
        bool has_scan_pose = false;
        SE3 scan_pose;
        SO3 scan_offset_R_lidar;
        Vec3d scan_offset_t_lidar = Vec3d::Zero();
        uint64_t scan_seq = 0;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            rate_hz = std::max(1e-3, pub_rate_hz_);
            do_pub_tf = pub_tf_;
            do_pub_scan = pub_scan_;
            do_pub_map = pub_map_;

            if (has_latest_state_) {
                const auto& state = latest_state_;
                tf_map_rot_lidar = state.rot_ * state.offset_R_lidar_;
                tf_map_pos_lidar = state.rot_ * state.offset_t_lidar_ + state.pos_;
                has_tf_pose = true;
            }

            if (has_latest_scan_) {
                scan_snapshot = std::make_shared<PointCloudType>();
                *scan_snapshot = *latest_scan_;
            }

            has_scan_pose = has_latest_scan_pose_;
            if (has_scan_pose) {
                scan_pose = latest_scan_pose_;
                scan_offset_R_lidar = latest_scan_offset_R_lidar_;
                scan_offset_t_lidar = latest_scan_offset_t_lidar_;
                scan_seq = latest_scan_seq_;

                tf_map_rot_lidar = scan_pose.so3() * scan_offset_R_lidar;
                tf_map_pos_lidar = scan_pose.so3() * scan_offset_t_lidar + scan_pose.translation();
                has_tf_pose = true;
            }

            if (do_pub_map && has_scan_pose && scan_snapshot != nullptr && !scan_snapshot->empty() &&
                scan_seq != last_history_scan_seq_) {
                auto scan_in_map = TransformScanToMap(*scan_snapshot, scan_pose, scan_offset_R_lidar, scan_offset_t_lidar);
                AppendHistoryScan(scan_in_map);
                last_history_scan_seq_ = scan_seq;
            }

            if (do_pub_map && merged_history_cloud_ != nullptr && !merged_history_cloud_->empty()) {
                history_snapshot = std::make_shared<PointCloudType>();
                *history_snapshot = *merged_history_cloud_;
            }
        }

        const auto stamp = node_->get_clock()->now();

        if (do_pub_tf && has_tf_pose && tf_broadcaster_ != nullptr) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = "map";
            tf_msg.child_frame_id = "lidar";

            tf_msg.transform.translation.x = tf_map_pos_lidar.x();
            tf_msg.transform.translation.y = tf_map_pos_lidar.y();
            tf_msg.transform.translation.z = tf_map_pos_lidar.z();
            tf_msg.transform.rotation.x = tf_map_rot_lidar.unit_quaternion().x();
            tf_msg.transform.rotation.y = tf_map_rot_lidar.unit_quaternion().y();
            tf_msg.transform.rotation.z = tf_map_rot_lidar.unit_quaternion().z();
            tf_msg.transform.rotation.w = tf_map_rot_lidar.unit_quaternion().w();
            tf_broadcaster_->sendTransform(tf_msg);
        }

        if (do_pub_scan && has_tf_pose && scan_snapshot != nullptr && current_scan_pub_ != nullptr) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*scan_snapshot, msg);
            msg.header.stamp = stamp;
            msg.header.frame_id = "lidar";
            current_scan_pub_->publish(msg);
        }

        if (do_pub_map && history_snapshot != nullptr && history_map_pub_ != nullptr) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*history_snapshot, msg);
            msg.header.stamp = stamp;
            msg.header.frame_id = "map";
            history_map_pub_->publish(msg);
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / rate_hz));
    }
}

}  // namespace lightning::ui
