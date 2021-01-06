//@Description
//@Author:wyz
//@Time:2020/12/17 下午7:08                            

#ifndef LIDAR_LOCALIZATION_LASER_SCAN_H
#define LIDAR_LOCALIZATION_LASER_SCAN_H
#include "sensors/sensor_interface.h"
#include "sensors/odom.h"
#include "sensors/point_type.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <deque>
#include <vector>

namespace fusion_localization {
#define CAST_TO_SCAN(x) std::static_pointer_cast<LaserScan>(x)

class LaserScan;
typedef std::shared_ptr<LaserScan> LaserScanPtr;

class LaserScan : public Sensor{
public:
    LaserScan(): Sensor(0){}
    explicit LaserScan(const LaserScanPtr& laser_scan_ptr, float resolution);
    explicit LaserScan(double timestamp): Sensor(timestamp){}
    LaserScan(double timestamp, std::vector<float>& ranges, std::vector<float>& intensities,float angle_min, float angle_max,
              float angle_increment, float time_increment, float range_min, float range_max);
    std::string Name() override;
    bool TransToCloud(PointTypes::CLOUD &point_cloud);
    bool TransToCloud(const Eigen::Matrix3d &pose, PointTypes::CLOUD& point_cloud);
    //TODO
    //@激光雷达运动畸变(原地改变数据)
    bool DistortRemove(const OdomPtr& odom1, const OdomPtr& odom2);
public:
    std::vector<float> ranges_;
    std::vector<float> intensities_;
    float angle_min_{};
    float angle_max_{};
    float angle_increment_{};
    float time_increment_{};
    float range_min_{};
    float range_max_{};

    PointTypes::CLOUD_PTR point_cloud_;
};

}


#endif //LIDAR_LOCALIZATION_LASER_SCAN_H
