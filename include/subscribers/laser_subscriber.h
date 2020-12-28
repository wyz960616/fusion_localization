//@Description
//@Author:wyz
//@Time:2020/12/18 上午9:40                            

#ifndef LIDAR_LOCALIZATION_LASER_SUBSCRIBER_H
#define LIDAR_LOCALIZATION_LASER_SUBSCRIBER_H
#include "sensors/sensor_factory.h"
#include "subscribers/subscriber.h"
#include <ros/ros.h>
namespace fusion_localization {
class LaserSubscriber : public Subscriber{
public:
    LaserSubscriber(const ros::NodeHandle& nh, std::string scan_topic_name, int buf_size);
    void LaserMsgCallBack(const sensor_msgs::LaserScanConstPtr &laser_msg);
};

}


#endif //LIDAR_LOCALIZATION_LASER_SUBSCRIBER_H
