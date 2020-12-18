//@Description
//@Author:wyz
//@Time:2020/12/18 上午9:41                            

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_H
#define LIDAR_LOCALIZATION_SUBSCRIBER_H
#include "sensors/sensor_interface.h"
#include <ros/ros.h>
#include <mutex>
#include <thread>

#define SUB_LOCK(x) std::lock_guard<std::mutex> lock(x);
namespace fusion_localization {
class Subscriber {
public:
    explicit Subscriber(const ros::NodeHandle& nh);

    virtual bool ParseData();
    virtual bool SyncedData(double synced_timestamp);

public:
    std::deque<SensorPtr> new_un_synced_data_;
    std::deque<SensorPtr> un_synced_data_;
    std::deque<SensorPtr> synced_data_;

protected:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::mutex mutex_;
    SensorPtr single_sensor_;
};

}


#endif //LIDAR_LOCALIZATION_SUBSCRIBER_H
