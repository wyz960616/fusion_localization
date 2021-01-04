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
    std::deque<SensorPtr> raw_data_;
    std::deque<SensorPtr> parsed_data_;

    std::deque<SensorPtr> synced_data_;
protected:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::mutex mutex_;

    /*!
     * 由于具体的sensor重载了函数SyncedData和SetMedianValue,所以每个类中需要添加一个单例，用于调用具体的重载函数
     */
    SensorPtr single_sensor_;
};

}


#endif //LIDAR_LOCALIZATION_SUBSCRIBER_H
