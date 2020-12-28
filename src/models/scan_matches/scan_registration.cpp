//@Description
//@Author:wyz
//@Time:2020/12/28 下午4:06                            

#include "models/scan_matches/scan_registration.h"


namespace fusion_localization {

void fusion_localization::ScanRegistration::SetParameters(const LaserScanPtr &current_laser_scan_ptr,
                                                          const LaserScanPtr &last_laser_scan_ptr,
                                                          const Eigen::Matrix4d &rough_estimate) {
    current_laser_scan_ptr_ = current_laser_scan_ptr;
    last_laser_scan_ptr_ = last_laser_scan_ptr;
    rough_estimate_ = rough_estimate;
}

void ScanRegistration::SetParameters(const LaserScanPtr &current_laser_scan_ptr, const Eigen::Matrix4d &rough_estimate) {
    current_laser_scan_ptr_ = current_laser_scan_ptr;
    rough_estimate_ = rough_estimate;
}
}
