//@Description
//@Author:wyz
//@Time:2021/1/15 下午3:12

#ifndef LIDAR_LOCALIZATION_NDT_H
#define LIDAR_LOCALIZATION_NDT_H
#include "models/scan_matches/scan_registration.h"
#include "models/scan_matches/local_map.h"
#include "pcl/registration/ndt_2d.h"
#include "sensors/point_type.h"
#include <yaml-cpp/yaml.h>

namespace fusion_localization {
class NdtRegistration : public ScanRegistration{
public:
    NdtRegistration(YAML::Node config_node);
    bool ScanToScan(Eigen::Matrix4d &precise_estimated);
    bool ScanToMap(Eigen::Matrix4d &precise_estimated);
    bool Match(Eigen::Matrix4d &precise_estimated) override;

private:
    int max_iteration_;
    float resolution_;
    bool scan2map_;
    PointTypes::CLOUD aligned_cloud_;
    std::shared_ptr<pcl::NormalDistributionsTransform2D<PointTypes::POINT_DATA, PointTypes::POINT_DATA>> ndt_ptr_;
//ScanToMap额外需要一个local_map
private:
    std::shared_ptr<LocalMap> local_map_;
    //n帧scan形成一张local_map
    int scan_size_{};
};
}


#endif //LIDAR_LOCALIZATION_NDT_H
