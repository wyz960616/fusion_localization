//@Description
//@Author:wyz
//@Time:2020/12/28 下午3:49                            

#ifndef LIDAR_LOCALIZATION_FRONT_END_H
#define LIDAR_LOCALIZATION_FRONT_END_H
#include "sensors/sensor_factory.h"
#include "models/scan_matches/scan_registration.h"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace fusion_localization {
class FrontEnd {
public:
    explicit FrontEnd(const YAML::Node& config_node);
    bool ReadData(const LaserScanPtr &current_scan_ptr, const LaserScanPtr &last_scan_ptr, const OdomPtr &current_odom, const ImuPtr &current_imu);
    bool ReadData(const LaserScanPtr &current_scan_ptr, const OdomPtr &current_odom, const ImuPtr &current_imu);
    bool Match(const LaserScanPtr &current_scan_ptr, const OdomPtr &current_odom, const ImuPtr &current_imu, Eigen::Matrix4d& result);
private:
    LaserScanPtr current_scan_ptr_;
    LaserScanPtr last_scan_ptr_;
    OdomPtr current_odom_;
    ImuPtr current_imu_;

    std::shared_ptr<ScanRegistration> scan_registration_;
};

}


#endif //LIDAR_LOCALIZATION_FRONT_END_H
