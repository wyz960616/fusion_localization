//@Description
//@Author:wyz
//@Time:2020/12/28 下午4:17                            

#ifndef LIDAR_LOCALIZATION_ICP_REGISTRATION_H
#define LIDAR_LOCALIZATION_ICP_REGISTRATION_H

#include "models/scan_matches/scan_registration.h"
#include "models/scan_matches/local_map.h"
#include <yaml-cpp/yaml.h>

namespace fusion_localization {
class ICPRegistration : public ScanRegistration{
public:
    ICPRegistration(const YAML::Node& config_node);
    void ScanToScan(const Eigen::Matrix4d &precise_estimated);
    void ScanToMap(const Eigen::Matrix4d &precise_estimated);
    bool Match(const Eigen::Matrix4d &precise_estimated) override;

private:
    int max_iteration_;
    float resolution_;
    bool scan2map_;
//ScanToMap额外需要一个local_map
private:
    std::shared_ptr<LocalMap> local_map_;
    //n帧scan形成一张local_map
    int scan_size_;
};

}


#endif //LIDAR_LOCALIZATION_ICP_REGISTRATION_H
