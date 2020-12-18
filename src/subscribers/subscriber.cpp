//@Description
//@Author:wyz
//@Time:2020/12/18 上午9:41                            

#include "subscribers/subscriber.h"

namespace fusion_localization {
Subscriber::Subscriber(const ros::NodeHandle& nh): nh_(nh) {

}

bool Subscriber::ParseData() {
    if(new_un_synced_data_.empty())
        return false;

    SUB_LOCK(mutex_)
    un_synced_data_.insert(un_synced_data_.end(),new_un_synced_data_.begin(),new_un_synced_data_.end());
    new_un_synced_data_.clear();
    return true;
}

bool Subscriber::SyncedData(double synced_timestamp) {
    if(single_sensor_->SyncData(un_synced_data_, synced_timestamp, synced_data_)) {
        return true;
    }
    return false;
}
}
