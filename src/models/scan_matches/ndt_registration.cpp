//@Description
//@Author:wyz
//@Time:2021/1/15 下午3:12

#include "models/scan_matches/ndt_registration.h"
#include "tools/pose_functions.h"
#include "global_definition.h"

namespace fusion_localization {

NdtRegistration::NdtRegistration(YAML::Node config_node) {
    max_iteration_ = config_node["MaximumIterations"].as<int>();
    Eigen::Vector2f GridCentre(config_node["GridCentre"][0].as<float>(),config_node["GridCentre"][1].as<float>());
    Eigen::Vector2f GridExtent(config_node["GridExtent"][0].as<float>(),config_node["GridExtent"][1].as<float>());
    Eigen::Vector2f GridStep(config_node["GridStep"][0].as<float>(),config_node["GridStep"][1].as<float>());
    Eigen::Vector3d OptimizationStepSize(config_node["OptimizationStepSize"][0].as<double>(),
                                         config_node["OptimizationStepSize"][1].as<double>(),
                                         config_node["OptimizationStepSize"][2].as<double>());
    double epsilon = config_node["TransformationEpsilon"].as<double>();
    ndt_ptr_ = std::make_shared<pcl::NormalDistributionsTransform2D<PointTypes::POINT_DATA, PointTypes::POINT_DATA>>();
    ndt_ptr_->setMaximumIterations (max_iteration_);
    ndt_ptr_->setGridCentre (GridCentre);
    ndt_ptr_->setGridExtent (GridExtent);
    ndt_ptr_->setGridStep (GridStep);
    ndt_ptr_->setOptimizationStepSize (OptimizationStepSize);
    ndt_ptr_->setTransformationEpsilon (epsilon);

    scan2map_ = config_node["scan2map"].as<bool>();
    resolution_ = config_node["resolution"].as<float>();

    if(scan2map_) {
        local_map_ = std::make_shared<LocalMap>();
    }
}

bool NdtRegistration::ScanToScan(Eigen::Matrix4d &precise_estimated) {
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
        ndt_ptr_->setInputSource(current_point_cloud);
        ndt_ptr_->setInputTarget(last_point_cloud);
        aligned_cloud_.clear();
        Eigen::Matrix4f e = rough_estimate_.cast<float>();
        ndt_ptr_->align(aligned_cloud_, e);
        precise_estimated = e.cast<double>();
    }

#ifdef ICP_PRINT
    {
        cv::Mat out;
        PointTypes::DrawCompare(*last_point_cloud, *current_point_cloud, aligned_cloud_, out);
        std::string path = WORK_SPACE_PATH + "/icp_compares/pcl_ndt_" + std::to_string(cloud_id) + "_compare.png";
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

bool NdtRegistration::Match(Eigen::Matrix4d &precise_estimated) {
    if(scan2map_) {
        return ScanToMap(precise_estimated);
    }else {
        return ScanToScan(precise_estimated);
    }
}

bool NdtRegistration::ScanToMap(Eigen::Matrix4d &precise_estimated) {
    return false;
}
}
