#pragma once

#include <atomic>
#include <condition_variable>
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

    void SetPublishOptions(bool pub_tf, bool pub_scan, bool pub_map, double map_voxel_leaf_size);

   private:
    CloudPtr TransformScanToMap(const PointCloudType& scan_lidar, const SE3& T_map_imu, const SO3& R_imu_lidar,
                                const Vec3d& t_imu_lidar) const;
    void DownsampleCloudInplace(const CloudPtr& cloud, double voxel_leaf_size) const;
    void PublishWorker();

   private:
    rclcpp::Node::SharedPtr node_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr history_map_pub_;

    std::mutex mtx_;
    struct TFTask {
        SO3 map_rot_lidar = SO3();
        Vec3d map_pos_lidar = Vec3d::Zero();
        rclcpp::Time stamp;
    };

    struct ScanTask {
        CloudPtr scan = nullptr;
        SE3 T_map_imu = SE3();
        SO3 R_imu_lidar = SO3();
        Vec3d t_imu_lidar = Vec3d::Zero();
        SO3 map_rot_lidar = SO3();
        Vec3d map_pos_lidar = Vec3d::Zero();
        bool publish_tf = true;
        bool publish_scan = true;
        bool publish_map = true;
        rclcpp::Time stamp;
    };

    std::deque<TFTask> tf_tasks_;
    std::deque<ScanTask> scan_tasks_;
    SO3 latest_offset_R_lidar_ = SO3();
    Vec3d latest_offset_t_lidar_ = Vec3d::Zero();

    CloudPtr global_map_cloud_ = std::make_shared<PointCloudType>();
    int scan_count_since_compact_ = 0;
    int global_map_compact_every_n_ = 300;

    bool pub_tf_ = true;
    bool pub_scan_ = true;
    bool pub_map_ = true;
    double map_voxel_leaf_size_ = 0.0;

    std::condition_variable cv_;

    std::atomic_bool exit_{false};
    std::thread thread_;
};

}  // namespace lightning::ui
