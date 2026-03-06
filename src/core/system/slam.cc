//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#if LIGHTNING_WITH_UI
#include "ui/pangolin_window.h"
#endif
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <exception>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace lightning {

namespace {

std::shared_ptr<PointCloudType> OccupancyGridToPointCloud(const nav_msgs::msg::OccupancyGrid& map) {
    auto cloud = std::make_shared<PointCloudType>();

    const uint32_t width = map.info.width;
    const uint32_t height = map.info.height;
    if (width == 0 || height == 0 || map.data.empty()) {
        cloud->width = 0;
        cloud->height = 1;
        cloud->is_dense = true;
        return cloud;
    }

    const size_t expected_cells = static_cast<size_t>(width) * static_cast<size_t>(height);
    const size_t cell_count = std::min(expected_cells, map.data.size());

    size_t occupied_count = 0;
    for (size_t i = 0; i < cell_count; ++i) {
        if (map.data[i] > 0) {
            ++occupied_count;
        }
    }
    cloud->reserve(occupied_count);

    const float resolution = static_cast<float>(map.info.resolution);
    const float origin_x = static_cast<float>(map.info.origin.position.x);
    const float origin_y = static_cast<float>(map.info.origin.position.y);
    const float origin_z = static_cast<float>(map.info.origin.position.z);

    for (size_t i = 0; i < cell_count; ++i) {
        const int8_t occupancy = map.data[i];
        if (occupancy <= 0) {
            continue;
        }

        const uint32_t x = static_cast<uint32_t>(i % width);
        const uint32_t y = static_cast<uint32_t>(i / width);

        PointType pt;
        pt.x = origin_x + (static_cast<float>(x) + 0.5f) * resolution;
        pt.y = origin_y + (static_cast<float>(y) + 0.5f) * resolution;
        pt.z = origin_z;
        pt.intensity = static_cast<float>(occupancy);
        pt.time = 0.0f;
        cloud->push_back(pt);
    }

    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
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
    const auto system_yaml = yaml["system"];
    options_.with_loop_closing_ = system_yaml["with_loop_closing"].as<bool>();
    options_.with_visualization_ = system_yaml["with_ui"].as<bool>();
    options_.with_2dvisualization_ = system_yaml["with_2dui"].as<bool>();
    options_.with_gridmap_ = system_yaml["with_g2p5"].as<bool>();
    options_.step_on_kf_ = system_yaml["step_on_kf"].as<bool>();

    if (system_yaml["pub_tf"]) {
        options_.pub_tf_ = system_yaml["pub_tf"].as<bool>();
    }
    if (system_yaml["pub_scan"]) {
        options_.pub_scan_ = system_yaml["pub_scan"].as<bool>();
    }
    if (system_yaml["pub_map"]) {
        options_.pub_map_ = system_yaml["pub_map"].as<bool>();
    }
    if (system_yaml["pub_g2p5_map"]) {
        options_.pub_g2p5_map_ = system_yaml["pub_g2p5_map"].as<bool>();
    }
    const double default_pub_rate_hz = options_.pub_rate_hz_ > 0.0 ? options_.pub_rate_hz_ : 20.0;
    if (options_.pub_rate_hz_ <= 0.0) {
        LOG(WARNING) << "invalid default system pub_rate_hz=" << options_.pub_rate_hz_
                     << ", fallback to " << default_pub_rate_hz;
        options_.pub_rate_hz_ = default_pub_rate_hz;
    }
    if (system_yaml["pub_rate_hz"]) {
        const double pub_rate_hz = system_yaml["pub_rate_hz"].as<double>();
        if (pub_rate_hz <= 0.0) {
            LOG(WARNING) << "invalid config system.pub_rate_hz=" << pub_rate_hz
                         << ", keep default " << default_pub_rate_hz;
        } else {
            options_.pub_rate_hz_ = pub_rate_hz;
        }
    }

    const int default_map_pub_kf_gap = std::max(1, options_.map_pub_kf_gap_);
    if (options_.map_pub_kf_gap_ <= 0) {
        LOG(WARNING) << "invalid default system map_pub_kf_gap=" << options_.map_pub_kf_gap_
                     << ", fallback to " << default_map_pub_kf_gap;
        options_.map_pub_kf_gap_ = default_map_pub_kf_gap;
    }
    if (system_yaml["map_pub_kf_gap"]) {
        const int map_pub_kf_gap = system_yaml["map_pub_kf_gap"].as<int>();
        if (map_pub_kf_gap <= 0) {
            LOG(WARNING) << "invalid config system.map_pub_kf_gap=" << map_pub_kf_gap
                         << ", keep default " << default_map_pub_kf_gap;
        } else {
            options_.map_pub_kf_gap_ = map_pub_kf_gap;
        }
    }

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
        lc_->SetLoopClosedCB([this]() {
            if (options_.with_gridmap_ && g2p5_ != nullptr) {
                g2p5_->RedrawGlobalMap();
            }
            MarkMapDirtyForPublish(true);
        });
    }

    if (options_.with_visualization_) {
#if LIGHTNING_WITH_UI
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();
        if (yaml["ui"] && yaml["ui"]["scans"]) {
            const int scan_keep_num = yaml["ui"]["scans"].as<int>();
            if (scan_keep_num > 0) {
                ui_->SetCurrentScanSize(scan_keep_num);
                LOG(INFO) << "ui keep scans: " << scan_keep_num;
            }
        }

        lio_->SetUI(ui_);
#else
        LOG(WARNING) << "with_ui=true in config, but this build disables UI (LIGHTNING_WITH_UI=OFF)";
#endif
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

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

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        if (options_.pub_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        }

        if (options_.pub_scan_) {
            current_scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/current_scan", 10);
        }

        if (options_.pub_map_) {
            global_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/global_map", 1);
        }

        if (options_.pub_g2p5_map_ && options_.with_gridmap_) {
            g2p5_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/g2p5_map", 1);
        }

        if (options_.pub_tf_ || options_.pub_scan_ || options_.pub_map_ ||
            (options_.pub_g2p5_map_ && options_.with_gridmap_)) {
            publish_thread_exit_ = false;
            publish_thread_ = std::thread(&SlamSystem::PublishLoop, this);
        }

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    publish_thread_exit_ = true;
    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }

    if (ui_) {
#if LIGHTNING_WITH_UI
        ui_->Quit();
#endif
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

    lio_->ProcessPointCloud2(cloud);
    if (!lio_->Run()) {
        return;
    }

    UpdatePublishSnapshots();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    MarkMapDirtyForPublish(false);

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
#if LIGHTNING_WITH_UI
        ui_->UpdateKF(cur_kf_);
#endif
    }
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    if (!lio_->Run()) {
        return;
    }

    UpdatePublishSnapshots();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    MarkMapDirtyForPublish(false);

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
#if LIGHTNING_WITH_UI
        ui_->UpdateKF(cur_kf_);
#endif
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

void SlamSystem::UpdatePublishSnapshots() {
    if (!(options_.pub_tf_ || options_.pub_scan_)) {
        return;
    }

    NavState latest_state = lio_->GetState();
    std::shared_ptr<const PointCloudType> latest_scan = lio_->GetScanUndistShared();

    std::lock_guard<std::mutex> lock(publish_snapshot_mutex_);
    latest_pub_state_ = latest_state;
    latest_pub_scan_ = latest_scan;
    has_latest_pub_state_ = true;
}

void SlamSystem::MarkMapDirtyForPublish(bool force_republish) {
    const size_t keyframe_count = lio_->GetKeyframeCount();

    std::lock_guard<std::mutex> lock(publish_snapshot_mutex_);
    latest_pub_kf_count_ = keyframe_count;
    map_pub_dirty_ = true;
    map_pub_force_republish_ = map_pub_force_republish_ || force_republish;
}

void SlamSystem::PublishLoop() {
    rclcpp::WallRate rate(options_.pub_rate_hz_);

    while (!publish_thread_exit_ && rclcpp::ok()) {
        try {
            NavState state_snapshot;
            std::shared_ptr<const PointCloudType> scan_snapshot = nullptr;
            bool has_state = false;
            bool should_pub_map = false;
            bool should_pub_global_map = false;
            bool should_pub_g2p5_map = false;
            bool force_republish = false;
            size_t map_pub_target_kf_count = 0;

            {
                std::lock_guard<std::mutex> lock(publish_snapshot_mutex_);
                has_state = has_latest_pub_state_;
                if (has_state) {
                    state_snapshot = latest_pub_state_;
                }
                scan_snapshot = latest_pub_scan_;

                should_pub_global_map = options_.pub_map_ && global_map_pub_ != nullptr;
                should_pub_g2p5_map = options_.pub_g2p5_map_ && options_.with_gridmap_ && g2p5_ != nullptr &&
                                      g2p5_map_pub_ != nullptr;
                if ((should_pub_global_map || should_pub_g2p5_map) && map_pub_dirty_) {
                    const size_t kf_gap = static_cast<size_t>(std::max(1, options_.map_pub_kf_gap_));
                    const bool reach_gap = latest_pub_kf_count_ >= published_map_kf_count_ + kf_gap;
                    force_republish = map_pub_force_republish_;
                    if (reach_gap || force_republish) {
                        should_pub_map = true;
                        map_pub_target_kf_count = latest_pub_kf_count_;
                    }
                }
            }

            int64_t stamp_ns = node_->get_clock()->now().nanoseconds();
            if (has_state && state_snapshot.timestamp_ > 0.0) {
                stamp_ns = static_cast<int64_t>(state_snapshot.timestamp_ * 1e9);
            }

            builtin_interfaces::msg::Time stamp;
            stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
            stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);

            if (options_.pub_tf_ && tf_broadcaster_ != nullptr && has_state) {
                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp = stamp;
                tf.header.frame_id = "map";
                tf.child_frame_id = "lidar";

                const SO3 map_rot_lidar = state_snapshot.rot_ * state_snapshot.offset_R_lidar_;
                const Vec3d map_pos_lidar = state_snapshot.rot_ * state_snapshot.offset_t_lidar_ + state_snapshot.pos_;
                const Eigen::Quaterniond q = map_rot_lidar.unit_quaternion();

                tf.transform.translation.x = map_pos_lidar.x();
                tf.transform.translation.y = map_pos_lidar.y();
                tf.transform.translation.z = map_pos_lidar.z();
                tf.transform.rotation.x = q.x();
                tf.transform.rotation.y = q.y();
                tf.transform.rotation.z = q.z();
                tf.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(tf);
            }

            if (options_.pub_scan_ && current_scan_pub_ != nullptr && scan_snapshot != nullptr) {
                sensor_msgs::msg::PointCloud2 scan_msg;
                pcl::toROSMsg(*scan_snapshot, scan_msg);
                scan_msg.header.stamp = stamp;
                scan_msg.header.frame_id = "lidar";
                current_scan_pub_->publish(scan_msg);
            }

            if (should_pub_map) {
                bool any_publish_attempt = false;
                bool any_publish_success = false;

                if (should_pub_global_map) {
                    any_publish_attempt = true;
                    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
                    if (global_map == nullptr || global_map->empty()) {
                        LOG(WARNING) << "skip map publish: global map is empty";
                    } else {
                        sensor_msgs::msg::PointCloud2 global_map_msg;
                        pcl::toROSMsg(*global_map, global_map_msg);
                        global_map_msg.header.stamp = stamp;
                        global_map_msg.header.frame_id = "map";
                        global_map_pub_->publish(global_map_msg);
                        any_publish_success = true;
                    }
                }

                if (should_pub_g2p5_map) {
                    any_publish_attempt = true;
                    auto g2p5_map = g2p5_->GetNewestMap();
                    if (g2p5_map == nullptr) {
                        LOG(WARNING) << "skip g2p5 map publish: g2p5 map is null";
                    } else {
                        const auto occupancy_map = g2p5_map->ToROS();
                        auto g2p5_cloud = OccupancyGridToPointCloud(occupancy_map);
                        if (g2p5_cloud->empty()) {
                            LOG(WARNING) << "skip g2p5 map publish: g2p5 map cloud is empty";
                        } else {
                            sensor_msgs::msg::PointCloud2 g2p5_map_msg;
                            pcl::toROSMsg(*g2p5_cloud, g2p5_map_msg);
                            g2p5_map_msg.header.stamp = stamp;
                            g2p5_map_msg.header.frame_id = "map";
                            g2p5_map_pub_->publish(g2p5_map_msg);
                            any_publish_success = true;
                        }
                    }
                }

                if (!any_publish_attempt) {
                    LOG(WARNING) << "skip map publish: map publishers unavailable";
                }

                if (any_publish_success) {
                    std::lock_guard<std::mutex> lock(publish_snapshot_mutex_);
                    published_map_kf_count_ = std::max(published_map_kf_count_, map_pub_target_kf_count);
                    map_pub_force_republish_ = false;
                }

                std::lock_guard<std::mutex> lock(publish_snapshot_mutex_);
                map_pub_dirty_ = map_pub_force_republish_ || (latest_pub_kf_count_ > published_map_kf_count_);
            }

            rate.sleep();
        } catch (const std::exception& e) {
            if (publish_thread_exit_) {
                break;
            }
            LOG(ERROR) << "publish loop exception: " << e.what();
            rate.sleep();
        } catch (...) {
            if (publish_thread_exit_) {
                break;
            }
            LOG(ERROR) << "publish loop exception: unknown";
            rate.sleep();
        }
    }
}

}  // namespace lightning
