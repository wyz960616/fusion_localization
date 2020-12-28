//@Description
//@Author:wyz
//@Time:2020/12/17 下午8:41                            

#ifndef LIDAR_LOCALIZATION_ODOMSUBSCRIBER_H
#define LIDAR_LOCALIZATION_ODOMSUBSCRIBER_H
#include "sensors/sensor_factory.h"
#include "subscribers/subscriber.h"
#include <ros/ros.h>
#include <mutex>
#include <thread>
namespace fusion_localization {
class OdomSubscriber: public Subscriber{
public:
    OdomSubscriber(const ros::NodeHandle&  nh, std::string odom_topic_name, int buf_size);
    void OdomMsgCallBack(const nav_msgs::OdometryConstPtr& odom_msg);
};

}


#endif //LIDAR_LOCALIZATION_ODOMSUBSCRIBER_H
