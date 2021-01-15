//@Description
//@Author:wyz
//@Time:2020/12/28 下午4:17                            

#include "tools/pose_functions.h"
#include "models/scan_matches/icp_registration.h"
#include "global_definition.h"
#include <pcl/io/pcd_io.h>

namespace fusion_localization {

ICPRegistration::ICPRegistration(YAML::Node config_node) {
    max_iteration_ = config_node["max_iteration"].as<int>();
    resolution_ = config_node["resolution"].as<float>();
    scan2map_ = config_node["scan2map"].as<bool>();
    if(!scan2map_) {
        double MaxCorrespondenceDistance = config_node["MaxCorrespondenceDistance"].as<double>();
        double EuclideanFitnessEpsilon = config_node["EuclideanFitnessEpsilon"].as<double>();
        double TransformationEpsilon = config_node["TransformationEpsilon"].as<double>();
        std::cout << "-------采用scan2scan匹配方法------" << std::endl;
        std::cout << "MaximumIterations:" <<  max_iteration_ << std::endl;
        std::cout << "MaxCorrespondenceDistance:" <<  MaxCorrespondenceDistance << std::endl;
        std::cout << "EuclideanFitnessEpsilon:" <<  EuclideanFitnessEpsilon << std::endl;
        std::cout << "TransformationEpsilon:" <<  TransformationEpsilon << std::endl;

        icp_ptr_ = std::make_shared<pcl::IterativeClosestPoint<PointTypes::POINT_DATA, PointTypes::POINT_DATA>>();
        icp_ptr_->setMaximumIterations(max_iteration_);
        icp_ptr_->setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
        icp_ptr_->setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
        icp_ptr_->setTransformationEpsilon(TransformationEpsilon);
    }else {

    }
    rough_estimate_.setIdentity();
}
/*!
 * @breif 根据配置文件中的参数选择 ScanToMap/ScanToScan
 * @param precise_estimated 雷达匹配之后位姿估计的返回值
 * @return 若匹配失败返回false
 */
bool ICPRegistration::Match(Eigen::Matrix4d &precise_estimated) {
    if(scan2map_) {
        return ScanToMap(precise_estimated);
    }else {
        return ScanToScan(precise_estimated);
    }
}

/*!
 * @breif 激光雷达帧与帧之间的匹配，调用此函数前需要设置好此次帧间匹配的laser_scan
 * @param precise_estimated 雷达匹配之后位姿估计的返回值
 * @return 若匹配失败返回false
 */
bool ICPRegistration::ScanToScan(Eigen::Matrix4d &precise_estimated) {
    static int cloud_id = -1;
    static bool first_scan = true;
    ++cloud_id;
    //根据分辨率得到新的雷达数据
    Eigen::Matrix3d rough_estimate_3d;
    if(!PoseFunctions::Homogeneous4dto3d(rough_estimate_,rough_estimate_3d)) {
        LOG(WARNING) << "初始的odom四维齐次坐标转换失败";
        return false;
    }
    current_laser_scan_ptr_ = std::make_shared<LaserScan>(current_laser_scan_ptr_, resolution_);
    current_laser_scan_ptr_->TransToCloud(rough_estimate_3d);
    if(first_scan) {
        last_laser_scan_ptr_ = current_laser_scan_ptr_;
        last_laser_scan_ptr_->R_ = rough_estimate_3d.block<2,2>(0,0);
        last_laser_scan_ptr_->t_ = rough_estimate_3d.block<2,1>(0,2);
        first_scan = false;
        return true;
    }
    PointTypes::CLOUD_PTR &last_point_cloud  = last_laser_scan_ptr_->point_cloud_;
    PointTypes::CLOUD_PTR &current_point_cloud = current_laser_scan_ptr_->point_cloud_;
    {
        icp_ptr_->setInputSource(current_point_cloud);
        icp_ptr_->setInputTarget(last_point_cloud);
        aligned_cloud_.clear();
        Eigen::Matrix4f e = rough_estimate_.cast<float>();
        icp_ptr_->align(aligned_cloud_, e);
        precise_estimated = e.cast<double>();
    }

#ifdef ICP_PRINT
    {
        cv::Mat out;
        PointTypes::DrawCompare(*last_point_cloud, *current_point_cloud, aligned_cloud_, out);
        std::string path = WORK_SPACE_PATH + "/icp_compares/pcl_icp_" + std::to_string(cloud_id) + "_compare.png";
        cv::imwrite(path, out);
    };
#endif

    Eigen::Matrix3d precise_estimated_3d;
    if(!PoseFunctions::Homogeneous4dto3d(precise_estimated,precise_estimated_3d)) {
        LOG(WARNING) << "icp变换后四维齐次坐标转换失败";
        return false;
    }
    current_laser_scan_ptr_->TransToCloud(precise_estimated_3d);
    last_laser_scan_ptr_ = current_laser_scan_ptr_;
    return true;
}

/*!
 * @breif 激光雷达帧与图之间的匹配，调用此函数前需要设置好此次帧间匹配的laser_scan
 * @param precise_estimated 雷达匹配之后位姿估计的返回值
 * @return 若匹配失败返回false
 */
bool ICPRegistration::ScanToMap(Eigen::Matrix4d &precise_estimated) {

}


}
