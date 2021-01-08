//@Description
//@Author:wyz
//@Time:2020/12/28 下午3:49                            

#include "mapping/front_end/front_end.h"
#include <iostream>
namespace fusion_localization {

FrontEnd::FrontEnd(const YAML::Node& config_node) {
    std::cout << "------------------初始化FrontEnd----------------------" << std::endl;
    std::string scan_match_name = config_node["scan_match_name"].as<std::string>();
    std::cout << "采用的前端配准方法:" << scan_match_name << std::endl;
    if(scan_match_name == "pcl_icp") {
        scan_registration_ = std::make_shared<ICPRegistration>(config_node["pcl_icp"]);
    }else if(scan_match_name == "pcl_ndt") {

    }else {
        LOG(ERROR) << "The scan_match method " << scan_match_name << "doesn't exist.";
    }
}

bool
FrontEnd::ReadData(const LaserScanPtr &current_scan_ptr, const LaserScanPtr &last_scan_ptr, const OdomPtr &current_odom,
                   const ImuPtr &current_imu) {
    last_scan_ptr_ = last_scan_ptr;
    return ReadData(current_scan_ptr, current_odom, current_imu);
}

bool FrontEnd::ReadData(const LaserScanPtr &current_scan_ptr, const OdomPtr &current_odom, const ImuPtr &current_imu) {
    if(fabs(current_scan_ptr->timestamp_ - current_odom->timestamp_) > 0.05) {
        LOG(WARNING) << "FrontEnd's data is not synced.";
        return false;
    }
    if(fabs(current_scan_ptr->timestamp_ - current_imu->timestamp_) > 0.05) {
        LOG(WARNING) << "FrontEnd's data is not synced.";
        return false;
    }
    current_scan_ptr_ = current_scan_ptr;
    current_odom_ = current_odom;
    current_imu_ = current_imu;

    return true;
}

bool FrontEnd::Match(const LaserScanPtr &current_scan_ptr, const OdomPtr &current_odom, const ImuPtr &current_imu, Eigen::Matrix4d& precise_estimated) {
    if(!ReadData(current_scan_ptr, current_odom, current_imu)) {
        return false;
    }

    Eigen::Matrix4d rough_estimate;
    rough_estimate.setIdentity();
    rough_estimate.block<3,3>(0,0) = current_odom_->q_.toRotationMatrix();
    rough_estimate.block<3,1>(0,3) = current_odom->t_;

    scan_registration_->SetParameters(current_scan_ptr, rough_estimate);
    scan_registration_->Match(precise_estimated);
}

}
