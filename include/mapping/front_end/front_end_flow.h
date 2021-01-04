//@Description
//@Author:wyz
//@Time:2020/12/28 下午3:47                            

#ifndef LIDAR_LOCALIZATION_FRONT_END_FLOW_H
#define LIDAR_LOCALIZATION_FRONT_END_FLOW_H
#include "sensors/sensor_factory.h"
#include "subscribers/imu_subscriber.h"
#include "subscribers/odom_subscriber.h"
#include "subscribers/laser_subscriber.h"
#include "publishers/odom_publihser.h"
#include "mapping/front_end/front_end.h"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <ros/ros.h>
namespace fusion_localization {
class FrontEndFlow {
public:
    explicit FrontEndFlow(ros::NodeHandle &nh, const std::string& config_path);
    bool ReadData();
    bool HasData();
    bool ValidData();
    void Publish();

    bool Run();
private:
    ros::NodeHandle nh_;
    std::shared_ptr<ImuSubscriber> imu_subscriber_;
    std::shared_ptr<LaserSubscriber> laser_subscriber_;
    std::shared_ptr<OdomSubscriber> odom_subscriber_;
    std::shared_ptr<OdomPublisher> laser_odom_publihser_;
    std::shared_ptr<FrontEnd> front_end_ptr_;


    ImuPtr imu_synced_;
    OdomPtr odom_synced_;
    LaserScanPtr laser_scan_undistorted_;
    OdomPtr laser_odom_;
    Eigen::Matrix4d precise_estimated_;
};

}


#endif //LIDAR_LOCALIZATION_FRONT_END_FLOW_H
