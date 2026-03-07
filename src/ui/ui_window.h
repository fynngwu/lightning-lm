#ifndef LIGHTNING_UI_WINDOW_H
#define LIGHTNING_UI_WINDOW_H

#include "common/eigen_types.h"
#include "common/keyframe.h"

namespace lightning {
namespace ui {

class UIWindow {
   public:
    virtual ~UIWindow() = default;

    virtual bool Init() = 0;
    virtual void Reset(const std::vector<Keyframe::Ptr>& keyframes) = 0;
    virtual void UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud) = 0;
    virtual void UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud) = 0;
    virtual void UpdateNavState(const NavState& state) = 0;
    virtual void UpdateRecentPose(const SE3& pose) = 0;
    virtual void UpdateScan(CloudPtr cloud, const SE3& pose) = 0;
    virtual void UpdateKF(std::shared_ptr<Keyframe> kf) = 0;
    virtual void Quit() = 0;
    virtual bool ShouldQuit() = 0;
    virtual void SetTImuLidar(const SE3& T_imu_lidar) = 0;
    virtual void SetCurrentScanSize(int max_size_of_scan) = 0;
};

}  // namespace ui
}  // namespace lightning

#endif  // LIGHTNING_UI_WINDOW_H
