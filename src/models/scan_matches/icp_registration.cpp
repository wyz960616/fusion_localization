//@Description
//@Author:wyz
//@Time:2020/12/28 下午4:17                            

#include "models/scan_matches/icp_registration.h"
#include "tools/pose_functions.h"
namespace fusion_localization {

ICPRegistration::ICPRegistration(const YAML::Node &config_node) {
    max_iteration_ = config_node["max_iteration"].as<int>();
    resolution_ = config_node["resolution"].as<float>();
    scan2map_ = config_node["scan2map"].as<bool>();

    rough_estimate_.setIdentity();
}

bool ICPRegistration::Match(Eigen::Matrix4d &precise_estimated) {
    if(scan2map_) {
        return ScanToMap(precise_estimated);
    }else {
        return ScanToScan(precise_estimated);
    }
}


bool ICPRegistration::ScanToScan(Eigen::Matrix4d &precise_estimated) {
    static bool first_scan = false;
    //根据分辨率得到新的雷达数据
    LaserScanPtr current_laser_scan_ptr_ = std::make_shared<LaserScan>(current_laser_scan_ptr_, resolution_);
    Eigen::Matrix3d rough_estimate_3d;
    if(!PoseFunctions::Homogeneous4dto3d(rough_estimate_,rough_estimate_3d)) {
        return false;
    }
    current_laser_scan_ptr_->TransToCloud(rough_estimate_3d, current_laser_scan_ptr_->point_cloud_);
    if(!first_scan) {
        last_laser_scan_ptr_ = current_laser_scan_ptr_;
        first_scan = true;
        return false;
    }
    const pcl::PointCloud<pcl::PointXYZI> &last_point_cloud  = last_laser_scan_ptr_->point_cloud_;
    const pcl::PointCloud<pcl::PointXYZI> &current_point_cloud = current_laser_scan_ptr_->point_cloud_;
    {

    }

    Eigen::Matrix3d precise_estimated_3d;
    if(!PoseFunctions::Homogeneous4dto3d(precise_estimated,precise_estimated_3d)) {
        return false;
    }
    current_laser_scan_ptr_->TransToCloud(precise_estimated_3d, current_laser_scan_ptr_->point_cloud_);
    last_laser_scan_ptr_ = current_laser_scan_ptr_;
    return true;
}

bool ICPRegistration::ScanToMap(Eigen::Matrix4d &precise_estimated) {

}


}
