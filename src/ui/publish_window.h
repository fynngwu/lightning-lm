#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ui/ui_window.h"

namespace lightning::ui {

class PublishWindow : public UIWindow {
   public:
    explicit PublishWindow(rclcpp::Node::SharedPtr node);
    ~PublishWindow() override;

    bool Init() override;
    void Reset(const std::vector<Keyframe::Ptr>& keyframes) override;
    void UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud) override;
    void UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud) override;
    void UpdateNavState(const NavState& state) override;
    void UpdateRecentPose(const SE3& pose) override;
    void UpdateScan(CloudPtr cloud, const SE3& pose) override;
    void UpdateKF(std::shared_ptr<Keyframe> kf) override;
    void Quit() override;
    bool ShouldQuit() override;
    void SetTImuLidar(const SE3& T_imu_lidar) override;
    void SetCurrentScanSize(int current_scan_size) override;

    void SetPublishOptions(bool pub_tf, bool pub_scan, bool pub_map, double pub_rate_hz);

   private:
    CloudPtr TransformScanToMap(const PointCloudType& scan_lidar, const SE3& T_map_imu, const SO3& R_imu_lidar,
                                const Vec3d& t_imu_lidar) const;
    void AppendHistoryScan(const CloudPtr& scan_in_map);
    void PublishLoop();

   private:
    rclcpp::Node::SharedPtr node_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr history_map_pub_;

    std::mutex mtx_;
    NavState latest_state_;
    bool has_latest_state_ = false;
    CloudPtr latest_scan_ = std::make_shared<PointCloudType>();
    bool has_latest_scan_ = false;
    SE3 latest_scan_pose_ = SE3();
    bool has_latest_scan_pose_ = false;
    SO3 latest_offset_R_lidar_ = SO3();
    Vec3d latest_offset_t_lidar_ = Vec3d::Zero();
    SO3 latest_scan_offset_R_lidar_ = SO3();
    Vec3d latest_scan_offset_t_lidar_ = Vec3d::Zero();
    uint64_t latest_scan_seq_ = 0;
    uint64_t last_history_scan_seq_ = 0;

    std::deque<CloudPtr> history_scans_;
    CloudPtr merged_history_cloud_ = std::make_shared<PointCloudType>();
    size_t history_scan_max_size_ = 200;
    bool history_need_rebuild_ = false;

    SE3 T_imu_lidar_ = SE3();

    bool pub_tf_ = true;
    bool pub_scan_ = true;
    bool pub_map_ = true;
    double pub_rate_hz_ = 20.0;

    std::atomic_bool exit_{false};
    std::thread thread_;
};

}  // namespace lightning::ui
