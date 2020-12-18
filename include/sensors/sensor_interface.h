//@Description
//@Author:wyz
//@Time:2020/12/17 下午6:58
#ifndef LIDAR_LOCALIZATION_SENSOR_INTERFACE_H
#define LIDAR_LOCALIZATION_SENSOR_INTERFACE_H
#include <memory>
#include <string>
#include <deque>
#include "glog/logging.h"
namespace fusion_localization {
class Sensor;
typedef std::shared_ptr<Sensor> SensorPtr;
class Sensor {
public:
    explicit Sensor(double timestamp);
    virtual bool SetMedianValue(const SensorPtr& front_ptr, const SensorPtr& back_ptr, double synced_timestamp,
                                       SensorPtr& median_ptr);
    virtual bool SyncData(std::deque<SensorPtr>& un_synced_data, double synced_timestamp,
                          std::deque<SensorPtr>& synced_data);
    virtual std::string Name();

    double timestamp_;

public:
    static void MedianScale(double front_timestamp, double back_timestamp, double synced_timestamp,
                                 double& front_scale, double& back_scale) {
        front_scale = ( synced_timestamp - front_timestamp ) / ( back_timestamp - front_timestamp );
        back_scale = ( back_timestamp - synced_timestamp ) / ( back_timestamp - front_timestamp );
    }
};

}


#endif //LIDAR_LOCALIZATION_SENSOR_INTERFACE_H
