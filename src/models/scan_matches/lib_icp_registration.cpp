//@Description
//@Author:wyz
//@Time:2021/1/14 上午11:38

#include "models/scan_matches/lib_icp_registration.h"
#include "tools/pose_functions.h"
#include "global_definition.h"
namespace fusion_localization {

int LibIcpRegistration::TransToDoubleVector(double* M, const LaserScanPtr &laser_ptr) {
    if (laser_ptr == nullptr) {
        return false;
    }
    int num;
    for (auto &range:laser_ptr->ranges_) {
        if(range ==  INFINITY || range > laser_ptr->range_max_ || range < laser_ptr->range_min_) {
            continue;
        }
        ++num;
    }
    M = (double*)calloc(dim_ * num, sizeof(double));

    int k = 0;
    for(int i = 0 ; i < laser_ptr->ranges_.size() ; ++i) {
        if(laser_ptr->ranges_[i] ==  INFINITY || laser_ptr->ranges_[i] > laser_ptr->range_max_ || laser_ptr->ranges_[i] < laser_ptr->range_min_) {
            continue;
        }
        float angle = laser_ptr->angle_min_ + laser_ptr->angle_increment_ * float(i);
        M[k*2 + 0] = laser_ptr->ranges_[i] * cos(angle);
        M[k*2 + 1] = laser_ptr->ranges_[i] * sin(angle);
        ++k;
    }
    if( k != num) {
        LOG(ERROR) << "有效点个数转换错误";
    }
    return k;
}

int LibIcpRegistration::TransToWord(double *M, int size,const Eigen::Matrix2d &R, const Eigen::Vector2d &t) {
    for(int i = 0 ; i < size ; ++i) {
        Eigen::Vector2d point(M[i*2 + 0], M[i*2 + 1]);
        point = R*point + t;
        M[i*2 + 0] = point.x();
        M[i*2 + 1] = point.y();
    }
}

bool LibIcpRegistration::ScanToScan(Eigen::Matrix4d &precise_estimated) {
    static bool first = true;
    static int cloud_id = -1;
    ++cloud_id;
    Eigen::Matrix3d rough_estimate_3d;
    if(!PoseFunctions::Homogeneous4dto3d(rough_estimate_,rough_estimate_3d)) {
        LOG(WARNING) << "初始的odom四维齐次坐标转换失败";
        return false;
    }

    if(first) {
        first = !first;
        last_laser_scan_ptr_ = current_laser_scan_ptr_;
        last_laser_scan_ptr_->R_ = rough_estimate_3d.block<2,2>(0,0);
        last_laser_scan_ptr_->t_ = rough_estimate_3d.block<2,1>(0,2);
        return true;
    }

    double *target, *source;
    int num1 = TransToDoubleVector(target, last_laser_scan_ptr_);
    TransToWord(target, num1, last_laser_scan_ptr_->R_, last_laser_scan_ptr_->t_);
    int num2 = TransToDoubleVector(source, current_laser_scan_ptr_);
    
    Matrix R = Matrix::eye(2);
    Matrix t(2,1);
    R.val[0][0] = rough_estimate_3d(0,0);
    R.val[0][1] = rough_estimate_3d(0,1);
    R.val[1][0] = rough_estimate_3d(1,0);
    R.val[1][1] = rough_estimate_3d(1,1);
    t.val[0][0] = rough_estimate_3d(0,2);
    t.val[1][0] = rough_estimate_3d(1,2);

    if(type_ == Point2Point) {
        IcpPointToPoint icp(target, num1, dim_);
        if(icp.fit(source, num2, R, t, outlier_) == 0) {
            return false;
        }
    } else if(type_ == Point2Plane) {
        IcpPointToPoint icp(target, num1, dim_);
        if(icp.fit(source, num2, R, t, outlier_) == 0) {
            return false;
        }
    }

    Eigen::Matrix3d precise_estimated_3d;
    precise_estimated_3d.setIdentity();
    precise_estimated_3d(0,0) = R.val[0][0];
    precise_estimated_3d(0,1) = R.val[0][1];
    precise_estimated_3d(1,0) = R.val[1][0];
    precise_estimated_3d(1,1) = R.val[1][1];
    precise_estimated_3d(0,2) = t.val[0][0];
    precise_estimated_3d(1,2) = t.val[1][0];


#ifdef ICP_PRINT
    {
        PointTypes::CLOUD_PTR last_point_cloud_ptr;
        last_laser_scan_ptr_->TransToCloud();
        last_point_cloud_ptr = last_laser_scan_ptr_->point_cloud_;

        PointTypes::CLOUD current_point_cloud;
        current_laser_scan_ptr_->TransToCloud(rough_estimate_3d,current_point_cloud);

        PointTypes::CLOUD aligned_cloud;
        current_laser_scan_ptr_->TransToCloud(precise_estimated_3d, aligned_cloud);

        cv::Mat out;
        PointTypes::DrawCompare(*last_point_cloud_ptr, current_point_cloud, aligned_cloud, out);
        std::string path = WORK_SPACE_PATH + "/icp_compares/lib_icp_" + std::to_string(cloud_id) + "_compare.png";
        cv::imwrite(path, out);
    };
#endif
    precise_estimated.setIdentity();
    precise_estimated.block<2,2>(0,0) = precise_estimated_3d.block<2,2>(0,0);
    precise_estimated.block<2,1>(0,3) = precise_estimated_3d.block<2,1>(0,2);
    return true;
}


bool LibIcpRegistration::Match(Eigen::Matrix4d &precise_estimated) {
    if(scan2map_) {
        return ScanToMap(precise_estimated);
    }else {
        return ScanToScan(precise_estimated);
    }
}

bool LibIcpRegistration::ScanToMap(Eigen::Matrix4d &precise_estimated) {
    return false;
}

LibIcpRegistration::LibIcpRegistration(YAML::Node config_node) {
    scan2map_ = config_node["scan2map"].as<bool>();
    outlier_ = config_node["outlier"].as<double>();
    dim_ = config_node["dim"].as<int>();
    std::string type = config_node["type"].as<std::string>();
    if(type == "Point2Point") {
        type_ = Point2Point;
    }else if(type == "Point2Plane") {
        type_ = Point2Plane;
    }else {
        LOG(WARNING) << "无效的libicp type,使用默认的 Point2Plane";
        type_ = Point2Plane;
    }
}
}
