//@Description
//@Author:wyz
//@Time:2020/12/17 下午7:08                            

#include "sensors/laser_scan.h"

#include <utility>

namespace fusion_localization {

std::string LaserScan::Name() {
    return "laser-scan-sensor";
}

 bool LaserScan::TransToCloud() {
     int size = ranges_.size();
     if(!size){
         return false;
     }
     point_cloud_.clear();
     for(int i = 0 ; i < size ; ++i) {
         pcl::PointXYZI point;
         float angle = angle_min_ + angle_increment_ * i;
         point.x = ranges_[i] * cos(angle);
         point.y = ranges_[i] * sin(angle);
         point.intensity = intensities_[i];
         point_cloud_.push_back(point);
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
                     range_max_(range_max){
    TransToCloud();
}

}
