//@Description
//@Author:wyz
//@Time:2020/12/18 下午12:13                            

#ifndef LIDAR_LOCALIZATION_LASER_PUBLISHER_H
#define LIDAR_LOCALIZATION_LASER_PUBLISHER_H
#include "sensors/laser_scan.h"
#include <ros/ros.h>
namespace fusion_localization {
class LaserPublisher {
public:
    LaserPublisher(const ros::NodeHandle& nh, const std::string& laser_topic, int buf_size);
    void Publish(const ScanPtr& laser_scan);
private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
};

}


#endif //LIDAR_LOCALIZATION_LASER_PUBLISHER_H
