//@Description
//@Author:wyz
//@Time:2020/12/18 下午1:36                            

#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_H
#define LIDAR_LOCALIZATION_DATA_PRETREAT_H
#include "subscribers/odom_subscriber.h"
#include "subscribers/laser_subscriber.h"
#include "publishers/laser_publisher.h"
#include "publishers/odom_publihser.h"

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
    std::shared_ptr<OdomPublihser> odom_publisher_;
    std::shared_ptr<LaserPublisher> laser_publisher_;

    OdomPtr synced_odom_;
    ScanPtr distorted_laser_;
    ScanPtr undistorted_laser_;
};

}


#endif //LIDAR_LOCALIZATION_DATA_PRETREAT_H
