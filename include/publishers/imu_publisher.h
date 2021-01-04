//@Description
//@Author:wyz
//@Time:2020/12/18 下午12:13                            

#ifndef LIDAR_LOCALIZATION_IMU_PUBLISHER_H
#define LIDAR_LOCALIZATION_IMU_PUBLISHER_H
#include "sensors/sensor_factory.h"
#include <ros/ros.h>

namespace fusion_localization {
class ImuPublisher {
public:
public:
    ImuPublisher(const ros::NodeHandle& nh, const std::string& imu_topic, int buf_size);
    void Publish(const ImuPtr & imu_ptr);
private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;

    sensor_msgs::Imu imu_ros_;
};

}


#endif //LIDAR_LOCALIZATION_IMU_PUBLISHER_H
