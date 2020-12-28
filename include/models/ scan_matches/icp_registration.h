//@Description
//@Author:wyz
//@Time:2020/12/28 下午4:17                            

#ifndef LIDAR_LOCALIZATION_ICP_REGISTRATION_H
#define LIDAR_LOCALIZATION_ICP_REGISTRATION_H
#include "models/scan_matches/scan_registration.h"

namespace fusion_localization {
class IcpRegistration : public ScanRegistration{
public:
    IcpRegistration(const YAML::Node& config_node);

private:
    int max_iteration_;
};

}


#endif //LIDAR_LOCALIZATION_ICP_REGISTRATION_H
