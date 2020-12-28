//@Description
//@Author:wyz
//@Time:2020/12/18 上午9:40                            

#include "subscribers/laser_subscriber.h"

namespace fusion_localization {

LaserSubscriber::LaserSubscriber(const ros::NodeHandle &nh, std::string scan_topic_name, int buf_size): Subscriber(nh){
    subscriber_ = nh_.subscribe(scan_topic_name, buf_size, &LaserSubscriber::LaserMsgCallBack, this);
    single_sensor_ = std::make_shared<LaserScan>();
}

void LaserSubscriber::LaserMsgCallBack(const sensor_msgs::LaserScanConstPtr &laser_msg) {
    LaserScanPtr new_scan_ptr = SensorFactory::CreateLaserScanPtr(laser_msg);
    {
        SUB_LOCK(mutex_);
        raw_data_.push_back(new_scan_ptr);
    }

#ifdef PRINT_TEST_INFO
    {
        SUB_LOCK(mutex_);
        auto last_ptr = CAST_TO_SCAN(new_un_synced_data_.back());
        std::cout.setf(std::ios::fixed, std::ios::floatfield);//十进制计数法，不是科学计数法
        std::cout.precision(10);
        std::cout << "时间戳: \t" <<last_ptr->timestamp_
                  << " scan 大小:\t" << last_ptr->laser_scan_const_ptr->ranges.size() << std::endl;
    }
#endif
}
}
