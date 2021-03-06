//@Description
//@Author:wyz
//@Time:2020/12/28 下午4:17                            

#ifndef LIDAR_LOCALIZATION_ICP_REGISTRATION_H
#define LIDAR_LOCALIZATION_ICP_REGISTRATION_H

#include "models/scan_matches/scan_registration.h"
#include "models/scan_matches/local_map.h"
#include "pcl/registration/icp.h"
#include "sensors/point_type.h"
#include <yaml-cpp/yaml.h>

namespace fusion_localization {
class ICPRegistration : public ScanRegistration{
public:
    ICPRegistration(YAML::Node config_node);
    bool ScanToScan(Eigen::Matrix4d &precise_estimated);
    bool ScanToMap(Eigen::Matrix4d &precise_estimated);
    bool Match(Eigen::Matrix4d &precise_estimated) override;

private:
    int max_iteration_;
    float resolution_;
    bool scan2map_;
    PointTypes::CLOUD aligned_cloud_;
    std::shared_ptr<pcl::IterativeClosestPoint<PointTypes::POINT_DATA, PointTypes::POINT_DATA>> icp_ptr_;
//ScanToMap额外需要一个local_map
private:
    std::shared_ptr<LocalMap> local_map_;
    //n帧scan形成一张local_map
    int scan_size_{};
};

}


#endif //LIDAR_LOCALIZATION_ICP_REGISTRATION_H
