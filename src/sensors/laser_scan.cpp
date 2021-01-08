//@Description
//@Author:wyz
//@Time:2020/12/17 下午7:08                            

#include "sensors/laser_scan.h"

#include <utility>

namespace fusion_localization {

std::string LaserScan::Name() {
    return "laser-scan-sensor";
}

 bool LaserScan::TransToCloud(PointTypes::CLOUD &point_cloud) {
     int size = ranges_.size();
     if(!size){
         return false;
     }
     point_cloud.clear();
     for(int i = 0 ; i < size ; ++i) {
         pcl::PointXYZI point;
         float angle = angle_min_ + angle_increment_ * i;
         point.x = ranges_[i] * cos(angle);
         point.y = ranges_[i] * sin(angle);
         point.z = 1.0;
         point.intensity = intensities_[i];
         point_cloud.push_back(point);
     }
     return true;
}

bool LaserScan::TransToCloud(const Eigen::Matrix3d &pose) {
    TransToCloud(pose, *point_cloud_);
}

bool LaserScan::TransToCloud(const Eigen::Matrix3d &pose, PointTypes::CLOUD& point_cloud) {
    int size = ranges_.size();
    if(!size) {
        return false;
    }
    point_cloud.clear();
    Eigen::Matrix2d Rotation = pose.block<2,2>(0,0);
    Eigen::Vector2d translation = pose.block<2,1>(0,2);
    for(int i = 0 ; i < size ; ++i) {
        pcl::PointXYZI point;
        float angle = angle_min_ + angle_increment_ * i;
        Eigen::Vector2d t;
        t.x() = ranges_[i] * cos(angle);
        t.y() = ranges_[i] * sin(angle);
        t = Rotation * t + translation;
        point.x = t.x();
        point.y = t.y();
        point.z = 1.0;
        point.intensity = intensities_[i];
        point_cloud.push_back(point);
    }

    return true;
}

LaserScan::LaserScan(double timestamp, std::vector<float>& ranges, std::vector<float>& intensities, float angle_min, float angle_max,
                     float angle_increment, float time_increment, float range_min, float range_max):
                     Sensor(timestamp),
                     ranges_(std::move(ranges)),
                     intensities_(std::move(intensities)),
                     angle_min_(angle_min),
                     angle_max_(angle_max),
                     angle_increment_(angle_increment),
                     time_increment_(time_increment),
                     range_min_(range_min),
                     range_max_(range_max),
                     point_cloud_(new PointTypes::CLOUD){
    //TransToCloud(*point_cloud_);
}

LaserScan::LaserScan(const LaserScanPtr& laser_scan_ptr, float resolution) : Sensor(laser_scan_ptr->timestamp_){
    for(auto& range:laser_scan_ptr->ranges_) {
        ranges_.push_back(range/resolution);
    }
    intensities_ =  laser_scan_ptr->intensities_;
    angle_min_ = laser_scan_ptr->angle_min_;
    angle_max_ = laser_scan_ptr->angle_max_;
    angle_increment_ = laser_scan_ptr->angle_increment_;
    time_increment_ = laser_scan_ptr->time_increment_;
    range_min_ = laser_scan_ptr->range_min_;
    range_max_ = laser_scan_ptr->range_max_;
    point_cloud_ = PointTypes::CLOUD_PTR(new PointTypes::CLOUD);
}


}
