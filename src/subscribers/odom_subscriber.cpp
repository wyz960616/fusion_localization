//@Description
//@Author:wyz
//@Time:2020/12/17 下午8:41                            

#include "subscribers/odom_subscriber.h"
#include <iostream>
namespace fusion_localization {

OdomSubscriber::OdomSubscriber(const ros::NodeHandle& nh, std::string odom_topic_name, int buf_size): Subscriber(nh){
    subscriber_ = nh_.subscribe(odom_topic_name, buf_size ,
                                     &OdomSubscriber::OdomMsgCallBack , this);
    single_sensor_ = std::make_shared<Odom>();
}

void OdomSubscriber::OdomMsgCallBack(const nav_msgs::OdometryConstPtr &odom_msg) {
    SensorPtr new_odom_ptr = SensorFactory::CreateOdomPtr(odom_msg);
    {
        SUB_LOCK(mutex_);
        new_un_synced_data_.push_back(new_odom_ptr);
    }

#ifdef PRINT_TEST_INFO
    {
        SUB_LOCK(mutex_);
        std::cout << new_un_synced_data_.back()->timestamp_ << std::endl
        <<CAST_TO_ODOM(new_un_synced_data_.back())->Pose()<< std::endl;
    }
#endif
    
}

}
