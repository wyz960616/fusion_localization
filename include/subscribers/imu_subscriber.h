//@Description
//@Author:wyz
//@Time:2020/12/17 下午9:21                            

#ifndef LIDAR_LOCALIZATION_IMU_SUBSCRIBER_H
#define LIDAR_LOCALIZATION_IMU_SUBSCRIBER_H
#include "sensors/imu.h"
#include "subscribers/subscriber.h"
#include <ros/ros.h>
#include <thread>
#include <mutex>
namespace fusion_localization {
class ImuSubscriber : public Subscriber{
public:
    ImuSubscriber(const ros::NodeHandle& nh, std::string imu_topic_name, int buf_size);
    void ImuMsgCallBack(const sensor_msgs::ImuConstPtr &imu_msg);

};

}


#endif //LIDAR_LOCALIZATION_IMU_SUBSCRIBER_H
