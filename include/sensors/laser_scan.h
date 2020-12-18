//@Description
//@Author:wyz
//@Time:2020/12/17 下午7:08                            

#ifndef LIDAR_LOCALIZATION_LASER_SCAN_H
#define LIDAR_LOCALIZATION_LASER_SCAN_H
#include "sensors/sensor_interface.h"
#include <sensor_msgs/LaserScan.h>
#include <deque>

namespace fusion_localization {
#define CAST_TO_SCAN(x) std::static_pointer_cast<LaserScan>(x)

class LaserScan;
typedef std::shared_ptr<LaserScan> ScanPtr;
class LaserScan : public Sensor{
public:
    LaserScan(): Sensor(0){}
    explicit LaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan_ptr);
    std::string Name() override;
    static void ToRosMsg(const ScanPtr& laser_scan, sensor_msgs::LaserScan& laser_scan_ros);
public:
    sensor_msgs::LaserScanConstPtr laser_scan_const_ptr;

};

}


#endif //LIDAR_LOCALIZATION_LASER_SCAN_H
