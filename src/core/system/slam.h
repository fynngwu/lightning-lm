//
// Created by xiang on 25-5-6.
//

#ifndef LIGHTNING_SLAM_H
#define LIGHTNING_SLAM_H

#include <atomic>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

#include "lightning/srv/save_map.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

class LaserMapping;  //  lio 前端
class LoopClosing;   // 回环检测
class SimpleYawInitializer;

namespace ui {
class UIWindow;
class PublishWindow;
}

namespace g2p5 {
class G2P5;
}

/**
 * SLAM 系统调用接口
 */
class SlamSystem {
   public:
    enum class OnlineInitState {
        RUNNING,
        WAIT_NEXT_SCAN_FOR_INIT,
        WAIT_NEXT_INITIALPOSE,
    };

    struct Options {
        Options() {}

        bool online_mode_ = true;  // 在线模式，在线模式下会起一些子线程来做异步处理

        bool with_cc_ = true;               // 是否需要带交叉验证
        bool with_gridmap_ = true;          // 是否需要2D栅格
        bool with_loop_closing_ = true;     // 是否需要回环检测
        bool with_visualization_ = true;    // 是否需要可视化UI
        bool with_2dvisualization_ = true;  // 是否需要2D可视化UI
        bool pub_tf_ = true;
        bool pub_scan_ = true;
        bool pub_map_ = true;
        double pub_map_voxel_leaf_size_ = 0.3;

        bool step_on_kf_ = true;  // 是否在关键帧处暂停p
    };

    using SaveMapService = srv::SaveMap;

    SlamSystem(Options options);
    ~SlamSystem();

    /// 初始化
    bool Init(const std::string& yaml_path);

    /// 对外部交互接口
    /// 开始建图，输入地图名称
    void StartSLAM(std::string map_name);

    /// 保存地图，默认保存至./data/地图名/ 下方
    void SaveMap(const std::string& path = "");

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    /// 实时模式下的spin
    void Spin();

   private:
    void HandleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msg);
    void TriggerDefaultInitialPoseOnce();
    void HandleKeyframeUpdate();
    void HandleOnlineInitLidar();
    void HandlePublishIVoxMapRequest(const std_msgs::msg::Bool::SharedPtr& msg);
    bool TryOnlineInitWithScan(CloudPtr scan_undist, double stamp_sec, const std::string& scan_source);
    void ApplyOnlineInitResult(const SE3& pose, double stamp_sec, CloudPtr local_map);
    void PublishCurrentIVoxMap(double stamp_sec);

    /// ros端保存地图的实现
    void SaveMap(const SaveMapService::Request::SharedPtr request, SaveMapService::Response::SharedPtr response);

    Options options_;
    std::atomic_bool running_ = false;

    rclcpp::Service<SaveMapService>::SharedPtr savemap_service_ = nullptr;

    std::string map_name_;  // 地图名

    std::shared_ptr<LaserMapping> lio_ = nullptr;       // lio 前端
    std::shared_ptr<LoopClosing> lc_ = nullptr;         // 回环检测
    std::shared_ptr<ui::UIWindow> ui_ = nullptr;  // ui
    std::shared_ptr<ui::PublishWindow> publisher_ui_ = nullptr;
    std::shared_ptr<g2p5::G2P5> g2p5_ = nullptr;        // 栅格地图

    Keyframe::Ptr cur_kf_ = nullptr;
    std::shared_ptr<SimpleYawInitializer> yaw_init_ = nullptr;
    bool enable_online_init_ = false;
    std::string online_init_map_path_;
    OnlineInitState online_init_state_ = OnlineInitState::RUNNING;
    SE3 pending_init_pose_;
    bool online_init_auto_requested_ = false;

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr publish_ivox_map_sub_ = nullptr;
};
}  // namespace lightning

#endif  // LIGHTNING_SLAM_H
