//@Description
//@Author:wyz
//@Time:2020/12/18 下午12:13                            

#include "publishers/laser_publisher.h"

namespace fusion_localization {

LaserPublisher::LaserPublisher(const ros::NodeHandle& nh, const std::string& laser_topic, int buf_size) {
    nh_ = nh;
    publisher_ = nh_.advertise<sensor_msgs::LaserScan>(laser_topic, buf_size);
}

void LaserPublisher::Publish(const ScanPtr &laser_scan) {
    sensor_msgs::LaserScan laser_scan_ros;
    LaserScan::ToRosMsg(laser_scan, laser_scan_ros);
    publisher_.publish(laser_scan_ros);
}
}
