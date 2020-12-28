//@Description
//@Author:wyz
//@Time:2020/12/18 上午9:41                            

#include "subscribers/subscriber.h"

namespace fusion_localization {
Subscriber::Subscriber(const ros::NodeHandle& nh): nh_(nh) {

}

bool Subscriber::ParseData() {
    if(raw_data_.empty())
        return false;

    SUB_LOCK(mutex_)
    pushed_data_.insert(pushed_data_.end(), raw_data_.begin(), raw_data_.end());
    raw_data_.clear();
    return true;
}

//使用虚函数，可以重载对齐函数，但在绝大多数情况下，基类Sensor中的SyncData已经能够满足需求
//一般需要重载的是设置对齐的函数SetMedianValue
bool Subscriber::SyncedData(double synced_timestamp) {
    if(single_sensor_->SyncData(pushed_data_, synced_timestamp, synced_data_)) {
        return true;
    }
    return false;
}
}
