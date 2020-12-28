//@Description
//@Author:wyz
//@Time:2020/12/28 下午4:06                            

#ifndef LIDAR_LOCALIZATION_SCAN_REGISTRATION_H
#define LIDAR_LOCALIZATION_SCAN_REGISTRATION_H

#include "sensors/laser_scan.h"
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
namespace fusion_localization {
class ScanRegistration {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ScanRegistration() = default;
    void SetParameters(const LaserScanPtr &current_laser_scan_ptr, const LaserScanPtr &last_laser_scan_ptr,
                 const Eigen::Matrix4d &rough_estimate);
    void SetParameters(const LaserScanPtr &current_laser_scan_ptr, const Eigen::Matrix4d &rough_estimate);
    bool Match(const Eigen::Matrix4d &precise_estimated);

protected:
    LaserScanPtr current_laser_scan_ptr_;
    LaserScanPtr last_laser_scan_ptr_;

    Eigen::Matrix4d rough_estimate_;

};

}


#endif //LIDAR_LOCALIZATION_SCAN_REGISTRATION_H
