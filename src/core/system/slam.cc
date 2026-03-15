//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "core/system/simple_yaw_initializer.h"
#include "ui/publish_window.h"
#if LIGHTNING_WITH_UI
#include "ui/pangolin_window.h"
#endif
#include "core/lightning_math.hpp"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iomanip>
#include <opencv2/opencv.hpp>

namespace lightning {

namespace {

ESKF::CovType MakeOnlineInitCovarianceLikeImuInit() {
    ESKF::CovType cov = ESKF::CovType::Identity();
    cov(6, 6) = cov(7, 7) = cov(8, 8) = 0.00001;
    cov(9, 9) = cov(10, 10) = cov(11, 11) = 0.00001;
    cov(15, 15) = cov(16, 16) = cov(17, 17) = 0.0001;
    cov(18, 18) = cov(19, 19) = cov(20, 20) = 0.001;
    cov(21, 21) = cov(22, 22) = 0.00001;
    return cov;
}

SE3 MakeTlidarImu(const NavState& state) {
    const SO3 R_lidar_imu = state.offset_R_lidar_.inverse();
    const Vec3d t_lidar_imu = -(R_lidar_imu * state.offset_t_lidar_);
    return SE3(R_lidar_imu, t_lidar_imu);
}

SE3 MakeMapLidarPose(const NavState& state) {
    return SE3(state.rot_ * state.offset_R_lidar_, state.rot_ * state.offset_t_lidar_ + state.pos_);
}

}  // namespace

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();
    options_.pub_tf_ = yaml["system"]["pub_tf"].as<bool>();
    options_.pub_scan_ = yaml["system"]["pub_scan"].as<bool>();
    options_.pub_map_ = yaml["system"]["pub_map"].as<bool>();
    options_.pub_map_voxel_leaf_size_ = yaml["system"]["pub_map_voxel_leaf_size"].as<double>();
    if (yaml["system"]["enable_online_init"]) {
        enable_online_init_ = yaml["system"]["enable_online_init"].as<bool>();
    }
    if (yaml["system"]["online_init_map_path"]) {
        online_init_map_path_ = yaml["system"]["online_init_map_path"].as<std::string>();
    } else if (enable_online_init_ && yaml["system"]["map_path"]) {
        online_init_map_path_ = yaml["system"]["map_path"].as<std::string>();
    }

    if (enable_online_init_) {
        if (online_init_map_path_.empty()) {
            LOG(ERROR) << "online init enabled but system.online_init_map_path is empty";
            return false;
        }

        yaw_init_ = std::make_shared<SimpleYawInitializer>();
        if (!yaw_init_->Init(yaml_path, online_init_map_path_)) {
            LOG(ERROR) << "failed to init online yaw initializer";
            return false;
        }
    }

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
#if LIGHTNING_WITH_UI
        LOG(INFO) << "slam with 3D UI";
        auto pangolin_ui = std::make_shared<ui::PangolinWindow>();
        ui_ = pangolin_ui;
        ui_->Init();

        lio_->SetUI(ui_);
#else
        LOG(WARNING) << "with_ui=true but LIGHTNING_WITH_UI is OFF, fallback to publish ui if enabled";
        options_.with_visualization_ = false;
#endif
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次重绘
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();

        rclcpp::QoS qos(10);
        // qos.best_effort();

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                ProcessIMU(imu);
            });

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        if (enable_online_init_) {
            initialpose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/initialpose", qos, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                    HandleInitialPose(msg);
                });
        }
        publish_ivox_map_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "lightning/publish_ivox_map", qos, [this](std_msgs::msg::Bool::SharedPtr msg) { HandlePublishIVoxMapRequest(msg); });

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        const bool use_publisher_ui = !options_.with_visualization_ && (options_.pub_tf_ || options_.pub_scan_ || options_.pub_map_);
        if (use_publisher_ui) {
            LOG(INFO) << "with_ui=false, use publish ui";
            publisher_ui_ = std::make_shared<ui::PublishWindow>(node_);
            publisher_ui_->SetPublishOptions(options_.pub_tf_, options_.pub_scan_, options_.pub_map_,
                                             options_.pub_map_voxel_leaf_size_);
            ui_ = publisher_ui_;
            if (!ui_->Init()) {
                LOG(ERROR) << "failed to init publish ui";
                return false;
            }
            lio_->SetUI(ui_);
        }

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }
    if (enable_online_init_ && !online_init_auto_requested_) {
        TriggerDefaultInitialPoseOnce();
    }
    if (enable_online_init_ && online_init_state_ == OnlineInitState::WAIT_NEXT_INITIALPOSE) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    if (enable_online_init_ && online_init_state_ != OnlineInitState::RUNNING) {
        HandleOnlineInitLidar();
        return;
    }

    if (!lio_->Run()) {
        return;
    }

    HandleKeyframeUpdate();
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }
    if (enable_online_init_ && !online_init_auto_requested_) {
        TriggerDefaultInitialPoseOnce();
    }
    if (enable_online_init_ && online_init_state_ == OnlineInitState::WAIT_NEXT_INITIALPOSE) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    if (enable_online_init_ && online_init_state_ != OnlineInitState::RUNNING) {
        HandleOnlineInitLidar();
        return;
    }

    if (!lio_->Run()) {
        return;
    }

    HandleKeyframeUpdate();
}

void SlamSystem::TriggerDefaultInitialPoseOnce() {
    if (!enable_online_init_ || online_init_auto_requested_) {
        return;
    }

    online_init_auto_requested_ = true;

    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    if (node_ != nullptr) {
        msg->header.stamp = node_->get_clock()->now();
    }
    msg->header.frame_id = "map";
    msg->pose.pose.orientation.w = 1.0;

    LOG(INFO) << "enable_online_init=true, auto triggering default /initialpose at zero pose";
    HandleInitialPose(msg);
}

void SlamSystem::HandleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msg) {
    if (!enable_online_init_ || !running_) {
        return;
    }

    online_init_auto_requested_ = true;

    const auto& pose_msg = msg->pose.pose;
    SE3 input_pose(
        Quatd(pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z),
        Vec3d(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z));
    pending_init_pose_ = input_pose;
    PoseRPYD pose_rpy = math::SE3ToRollPitchYaw(pending_init_pose_);
    if (publisher_ui_) {
        publisher_ui_->PublishPoseTF(pending_init_pose_, "online_init_request");
    }
    cur_kf_ = nullptr;

    CloudPtr latest_scan;
    double latest_stamp = 0.0;
    if (lio_->GetLatestUndistortedScanForExternalInit(latest_scan, latest_stamp)) {
        LOG(INFO) << "received /initialpose, trying latest undistorted scan for online init immediately, xyz="
                  << pending_init_pose_.translation().transpose() << ", yaw=" << pose_rpy.yaw
                  << ", scan_stamp=" << std::setprecision(14) << latest_stamp;
        if (TryOnlineInitWithScan(latest_scan, latest_stamp, "latest undistorted scan")) {
            return;
        }

        online_init_state_ = OnlineInitState::WAIT_NEXT_INITIALPOSE;
        LOG(WARNING) << "online init failed with latest undistorted scan, waiting for next /initialpose";
        return;
    }

    online_init_state_ = OnlineInitState::WAIT_NEXT_SCAN_FOR_INIT;
    LOG(INFO) << "received /initialpose, no cached undistorted scan available, waiting next undistorted scan for online init, xyz="
              << pending_init_pose_.translation().transpose() << ", yaw=" << pose_rpy.yaw;
}

void SlamSystem::HandleOnlineInitLidar() {
    if (online_init_state_ == OnlineInitState::WAIT_NEXT_INITIALPOSE) {
        return;
    }

    CloudPtr scan_undist;
    double stamp_sec = 0.0;
    if (!lio_->ComputeUndistortedScanForExternalInit(scan_undist, stamp_sec)) {
        return;
    }

    if (!TryOnlineInitWithScan(scan_undist, stamp_sec, "next undistorted scan")) {
        online_init_state_ = OnlineInitState::WAIT_NEXT_INITIALPOSE;
        LOG(WARNING) << "online init failed with next undistorted scan, waiting for next /initialpose";
    }
}

void SlamSystem::HandlePublishIVoxMapRequest(const std_msgs::msg::Bool::SharedPtr& msg) {
    if (!running_ || msg == nullptr || !msg->data) {
        return;
    }

    double stamp_sec = lio_->GetState().timestamp_;
    if (stamp_sec <= 0.0) {
        stamp_sec = lio_->latest_undistorted_stamp_sec_;
    }
    PublishCurrentIVoxMap(stamp_sec);
}

bool SlamSystem::TryOnlineInitWithScan(CloudPtr scan_undist, double stamp_sec, const std::string& scan_source) {
    if (scan_undist == nullptr || scan_undist->empty()) {
        return false;
    }

    SE3 loc_pose;
    double score = 0.0;
    CloudPtr local_map;
    if (!yaw_init_->InitOnce(scan_undist, pending_init_pose_, loc_pose, score, local_map)) {
        return false;
    }

    ApplyOnlineInitResult(loc_pose, stamp_sec, local_map);
    online_init_state_ = OnlineInitState::RUNNING;
    PoseRPYD result_rpy = math::SE3ToRollPitchYaw(loc_pose);
    LOG(INFO) << "online init succeeded with " << scan_source << ", score: " << score
              << ", xyz=" << loc_pose.translation().transpose() << ", yaw=" << result_rpy.yaw;
    return true;
}

void SlamSystem::ApplyOnlineInitResult(const SE3& pose, double stamp_sec, CloudPtr local_map) {
    NavState state = lio_->kf_.GetX();
    state.SetPose(pose * MakeTlidarImu(state));
    state.SetVel(Vec3d::Zero());
    state.timestamp_ = stamp_sec;

    ESKF::CovType cov = MakeOnlineInitCovarianceLikeImuInit();
    lio_->kf_.ChangeX(state);
    lio_->kf_.ChangeP(cov);
    lio_->kf_.ChangeStamp(stamp_sec);

    lio_->kf_imu_.ChangeX(state);
    lio_->kf_imu_.ChangeP(cov);
    lio_->kf_imu_.ChangeStamp(stamp_sec);

    lio_->state_point_ = state;
    lio_->last_kf_ = nullptr;
    lio_->ivox_ = std::make_shared<LaserMapping::IVoxType>(lio_->ivox_options_);
    if (local_map != nullptr && !local_map->empty()) {
        lio_->ivox_->AddPoints(local_map->points);
    }
    lio_->flg_first_scan_ = false;
    lio_->skip_lidar_cnt_ = 0;
    cur_kf_ = nullptr;

    if (ui_) {
        ui_->UpdateNavState(state);
    }
    if (publisher_ui_) {
        const SE3 map_lidar = MakeMapLidarPose(state);
        publisher_ui_->PublishPoseTF(map_lidar, "lidar");
        publisher_ui_->PublishPoseTF(map_lidar, "online_init_output_result");
        publisher_ui_->PublishPoseTF(MakeMapLidarPose(state), "online_init_result");
        PublishCurrentIVoxMap(0.0);
    }
}

void SlamSystem::PublishCurrentIVoxMap(double stamp_sec) {
    (void)stamp_sec;
    if (publisher_ui_ == nullptr || lio_ == nullptr || lio_->ivox_ == nullptr) {
        return;
    }

    auto ivox_cloud = std::make_shared<PointCloudType>();
    ivox_cloud->points = lio_->ivox_->GetAllPoints();
    ivox_cloud->width = static_cast<uint32_t>(ivox_cloud->points.size());
    ivox_cloud->height = 1;
    ivox_cloud->is_dense = false;
    publisher_ui_->PublishIVoxMap(ivox_cloud);
}

void SlamSystem::HandleKeyframeUpdate() {
    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
