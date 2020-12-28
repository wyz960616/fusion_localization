//@Description
//@Author:wyz
//@Time:2020/12/28 下午3:49                            

#ifndef LIDAR_LOCALIZATION_FRONT_END_H
#define LIDAR_LOCALIZATION_FRONT_END_H
#include "sensors/laser_scan.h"
#include "sensors/odom.h"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace fusion_localization {
class FrontEnd {
public:
    FrontEnd(const std::string &config_path);
    bool ReadData();


private:
    LaserScanPtr current_scan_ptr_;
    LaserScanPtr last_scan_ptr_;
    OdomPtr current_odom_;
};

}


#endif //LIDAR_LOCALIZATION_FRONT_END_H
