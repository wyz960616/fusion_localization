//@Description
//@Author:wyz
//@Time:2020/12/18 下午12:13                            

#ifndef LIDAR_LOCALIZATION_LASER_PUBLISHER_H
#define LIDAR_LOCALIZATION_LASER_PUBLISHER_H
#include "sensors/sensor_factory.h"
#include <ros/ros.h>
namespace fusion_localization {
class LaserPublisher {
public:
    LaserPublisher(const ros::NodeHandle& nh, const std::string& laser_topic, int buf_size, std::string frame_id);
    void Publish(const LaserScanPtr& laser_scan);
private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;

    sensor_msgs::LaserScan laser_scan_ros_;
};

}


#endif //LIDAR_LOCALIZATION_LASER_PUBLISHER_H
