//@Description
//@Author:wyz
//@Time:2020/12/17 下午6:58

#include "sensors/sensor_interface.h"
namespace fusion_localization {
std::string Sensor::Name() {
    return "sensor-interface";
}
Sensor::Sensor(double timestamp) {
    timestamp_ = timestamp;
}
bool Sensor::SetMedianValue(const SensorPtr& front_ptr, const SensorPtr& back_ptr, double synced_timestamp,
                            SensorPtr& median_ptr) {
    double front_scale = 0.5, back_scale = 0.5;
    MedianScale(front_ptr->timestamp_, back_ptr->timestamp_,synced_timestamp,
                                 front_scale, back_scale);
    LOG(INFO) << "The " << Name() << "SetMedianValue do nothing.";
    return false;
}
bool Sensor::SyncData(std::deque<SensorPtr>& un_synced_data, double synced_timestamp,
                      std::deque<SensorPtr>& synced_data) {
    while (un_synced_data.size() >= 2) {
        if (un_synced_data.front()->timestamp_ > synced_timestamp)
            return false;
        if (un_synced_data.at(1)->timestamp_ < synced_timestamp) {
            un_synced_data.pop_front();
            continue;
        }
        if (synced_timestamp - un_synced_data.front()->timestamp_ > 0.2) {
            un_synced_data.pop_front();
            return false;
        }
        if (un_synced_data.at(1)->timestamp_ - synced_timestamp > 0.2) {
            un_synced_data.pop_front();
            return false;
        }
        break;
    }
    //TODO
    //属于另外一种情况，对齐的数据还没到，可以额外添加这种标志位
    if (un_synced_data.size() < 2) {
        return false;
    }

    SensorPtr front_senor_ptr = un_synced_data.at(0);
    SensorPtr last_sensor_ptr = un_synced_data.at(1);

    SensorPtr median_sensor_ptr;
    if(!SetMedianValue(front_senor_ptr, last_sensor_ptr, synced_timestamp, median_sensor_ptr)) {
        return false;
    }
    synced_data.push_back(median_sensor_ptr);
    return true;
}


}