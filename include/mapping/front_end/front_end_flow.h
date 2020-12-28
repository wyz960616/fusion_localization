//@Description
//@Author:wyz
//@Time:2020/12/28 下午3:47                            

#ifndef LIDAR_LOCALIZATION_FRONT_END_FLOW_H
#define LIDAR_LOCALIZATION_FRONT_END_FLOW_H
#include "sensors/sensor_factory.h"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>

namespace fusion_localization {
class FrontEndFlow {
    FrontEndFlow(const std::string& config_path);
};

}


#endif //LIDAR_LOCALIZATION_FRONT_END_FLOW_H
