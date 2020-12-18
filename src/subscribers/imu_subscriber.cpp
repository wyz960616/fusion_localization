//@Description
//@Author:wyz
//@Time:2020/12/17 下午9:21                            

#include "subscribers/imu_subscriber.h"

namespace fusion_localization {

ImuSubscriber::ImuSubscriber(const ros::NodeHandle& nh, std::string imu_topic_name, int buf_size): Subscriber(nh){
    subscriber_ = nh_.subscribe(imu_topic_name, buf_size, &ImuSubscriber::ImuMsgCallBack, this);
    single_sensor_ = std::make_shared<Imu>();
}

void ImuSubscriber::ImuMsgCallBack(const sensor_msgs::ImuConstPtr &imu_msg) {
    ImuPtr new_imu_ptr = std::make_shared<Imu>(imu_msg);
    {
        SUB_LOCK(mutex_)
        new_un_synced_data_.push_back(new_imu_ptr);
    }
#ifdef PRINT_TEST_INFO
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto imu_ptr = CAST_TO_IMU(new_un_synced_data_.back());
        std::cout << imu_ptr->Name() << "时间戳: "<<imu_ptr->timestamp_ << std::endl;
    }
#endif
}
}
