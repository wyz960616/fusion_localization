//@Description
//@Author:wyz
//@Time:2020/12/18 上午9:41                            

#include "subscribers/subscriber.h"

namespace fusion_localization {
Subscriber::Subscriber(const ros::NodeHandle& nh): nh_(nh) {

}
/*!
 * @breif 多线程加锁读取数据
 * 采用ROS的subscribe机制，将数据读取到raw_data_队列中，为保障数据同步，加锁将数据解析到pushed_data_中
 * @parameters raw_data_ 原始数据队列
 * @parameters parsed_data_ 解析后的数据队列
 * @return 若不存在数据返回false
 */
bool Subscriber::ParseData() {
    if(raw_data_.empty())
        return false;

    SUB_LOCK(mutex_)
    parsed_data_.insert(parsed_data_.end(), raw_data_.begin(), raw_data_.end());
    raw_data_.clear();
    return true;
}

/*!
 * @breif 根据时间戳完成数据对齐
 * @param synced_timestamp
 * @return
 */
bool Subscriber::SyncedData(double synced_timestamp) {
    if(single_sensor_->SyncData(parsed_data_, synced_timestamp, synced_data_)) {
        return true;
    }
    return false;
}
}
