//@Description
//@Author:wyz
//@Time:2020/12/18 下午12:13                            

#include "publishers/laser_publisher.h"

#include <utility>

namespace fusion_localization {

LaserPublisher::LaserPublisher(const ros::NodeHandle& nh, const std::string& laser_topic, int buf_size, std::string frame_id) {
    nh_ = nh;
    publisher_ = nh_.advertise<sensor_msgs::LaserScan>(laser_topic, buf_size);
    frame_id_ = std::move(frame_id);
}

void LaserPublisher::Publish(const LaserScanPtr &laser_scan) {
    SensorFactory::LaserScanToRosMsg(laser_scan, laser_scan_ros_);
    publisher_.publish(laser_scan_ros_);
}
}
