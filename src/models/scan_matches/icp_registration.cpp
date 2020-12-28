//@Description
//@Author:wyz
//@Time:2020/12/28 下午4:17                            

#include "models/scan_matches/icp_registration.h"

namespace fusion_localization {

ICPRegistration::ICPRegistration(const YAML::Node &config_node) {
    max_iteration_ = config_node["max_iteration"].as<int>();
    resolution_ = config_node["resolution"].as<float>();
    scan2map_ = config_node["scan2map"].as<bool>();

    rough_estimate_.setIdentity();
}

bool ICPRegistration::Match(const Eigen::Matrix4d &precise_estimated) {
    if(scan2map_) {
        ScanToMap(precise_estimated);
    }else {
        ScanToScan(precise_estimated);
    }
}


void ICPRegistration::ScanToScan(const Eigen::Matrix4d &precise_estimated) {
    static bool first_scan = false;
    //根据分辨率得到新的雷达数据
    //得到当前分辨率下的点云数据
    LaserScanPtr current_laser_scan_ptr_ = std::make_shared<LaserScan>(current_laser_scan_ptr_, resolution_);
    current_laser_scan_ptr_->TransToCloud(rough_estimate_, current_laser_scan_ptr_->point_cloud_);
    if(!first_scan) {
        last_laser_scan_ptr_ = current_laser_scan_ptr_;
        first_scan = true;
        return;
    }
    const pcl::PointCloud<pcl::PointXYZI> &last_point_cloud  = last_laser_scan_ptr_->point_cloud_;
    const pcl::PointCloud<pcl::PointXYZI> &current_point_cloud = current_laser_scan_ptr_->point_cloud_;
    {

    }

    current_laser_scan_ptr_->TransToCloud(precise_estimated, current_laser_scan_ptr_->point_cloud_);
    last_laser_scan_ptr_ = current_laser_scan_ptr_;
}

void ICPRegistration::ScanToMap(const Eigen::Matrix4d &precise_estimated) {

}


}
