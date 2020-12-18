//@Description
//@Author:wyz
//@Time:2020/12/17 下午7:08                            

#include "sensors/laser_scan.h"

namespace fusion_localization {
LaserScan::LaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan_ptr):Sensor(laser_scan_ptr->header.stamp.toSec()),
                                                                     laser_scan_const_ptr(laser_scan_ptr){

}

std::string LaserScan::Name() {
    return "laser-scan-sensor";
}

}
