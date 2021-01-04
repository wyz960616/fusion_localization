//@Description
//@Author:wyz
//@Time:2020/12/18 下午12:13                            

#include "publishers/imu_publisher.h"

namespace fusion_localization {

ImuPublisher::ImuPublisher(const ros::NodeHandle &nh, const std::string &imu_topic, int buf_size) {
    nh_ = nh;
    publisher_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, buf_size);
}

void ImuPublisher::Publish(const ImuPtr &imu_ptr) {
    SensorFactory::ImuToRosMsg(imu_ptr, imu_ros_);
    publisher_.publish(imu_ros_);
}

}
