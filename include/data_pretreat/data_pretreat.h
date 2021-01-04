//@Description
//@Author:wyz
//@Time:2020/12/18 下午1:36                            

#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_H
#define LIDAR_LOCALIZATION_DATA_PRETREAT_H
#include "subscribers/odom_subscriber.h"
#include "subscribers/laser_subscriber.h"
#include "subscribers/imu_subscriber.h"
#include "publishers/laser_publisher.h"
#include "publishers/odom_publihser.h"
#include "publishers/imu_publisher.h"

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>


namespace fusion_localization {
class DataPretreat {
public:
    DataPretreat(const ros::NodeHandle& nh, const std::string& config_path);
    void InitParameters(const YAML::Node& config_node);
    void LaserDistortionRemove();
    void Run();
    bool ReadData();
    bool HasSyncedData();
    bool ValidData();
    void Publish();
private:
    ros::NodeHandle nh_;
    std::shared_ptr<OdomSubscriber> odom_subscriber_;
    std::shared_ptr<LaserSubscriber> laser_subscriber_;
    std::shared_ptr<ImuSubscriber> imu_subscriber_;
    std::shared_ptr<OdomPublisher> odom_publisher_;
    std::shared_ptr<LaserPublisher> laser_publisher_;
    std::shared_ptr<ImuPublisher> imu_publisher_;

    OdomPtr synced_odom_;
    ImuPtr synced_imu_;
    LaserScanPtr distorted_laser_;
    LaserScanPtr undistorted_laser_;
};

}


#endif //LIDAR_LOCALIZATION_DATA_PRETREAT_H
