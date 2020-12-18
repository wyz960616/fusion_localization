//@Description
//@Author:wyz
//@Time:2020/12/18 下午1:36                            

#include "data_pretreat/data_pretreat.h"

namespace fusion_localization {

DataPretreat::DataPretreat(const ros::NodeHandle &nh, const std::string& config_path) {
    nh_ = nh;
    YAML::Node config_node = YAML::LoadFile(config_path);
    InitParameters(config_node);

}

void DataPretreat::InitParameters(const YAML::Node &config_node) {
    std::string odom_sub_topic = config_node["odom"]["sub_topic"].as<std::string>();
    int odom_sub_buf_size = config_node["odom"]["sub_buf_size"].as<int>();
    odom_subscriber_ = std::make_shared<OdomSubscriber>(nh_, odom_sub_topic, odom_sub_buf_size);

    std::string odom_pub_topic = config_node["odom"]["pub_topic"].as<std::string>();
    std::string odom_frame_id = config_node["odom"]["frame_id"].as<std::string>();
    std::string odom_child_frame_id = config_node["odom"]["child_frame_id"].as<std::string>();
    int odom_pub_buf_size = config_node["odom"]["pub_buf_size"].as<int>();
    odom_publisher_ = std::make_shared<OdomPublihser>(nh_, odom_pub_topic, odom_pub_buf_size,
                                                      odom_frame_id, odom_child_frame_id);

    std::string laser_sub_topic = config_node["laser"]["sub_topic"].as<std::string>();
    int laser_sub_buf_size = config_node["laser"]["sub_buf_size"].as<int>();
    laser_subscriber_ = std::make_shared<LaserSubscriber>(nh_, laser_sub_topic, laser_sub_buf_size);

    std::string laser_pub_topic = config_node["laser"]["pub_topic"].as<std::string>();
    int laser_pub_buf_size = config_node["laser"]["pub_buf_size"].as<int>();
    laser_publisher_ = std::make_shared<LaserPublisher>(nh_, laser_pub_topic, laser_pub_buf_size);
}

bool DataPretreat::ReadData() {
    static std::deque<SensorPtr>& un_synced_laser_data = laser_subscriber_->un_synced_data_;
    static std::deque<SensorPtr>& un_synced_odom_data = odom_subscriber_->un_synced_data_;
    static std::deque<SensorPtr>& laser_data = laser_subscriber_->synced_data_;
    static std::deque<SensorPtr>& synced_odom_data = odom_subscriber_->synced_data_;

    //从subscriber（带锁）取出数据到un_synced_data
    laser_subscriber_->ParseData();
    odom_subscriber_->ParseData();
    if(un_synced_laser_data.empty()) {
        return false;
    }
    //以激光雷达数据时间为对齐点，所以它不存在插值
    laser_data.push_back(un_synced_laser_data.front());
    un_synced_laser_data.pop_front();
    //时间戳对齐
    double scan_time_stamp = laser_data.front()->timestamp_;
    bool odom_synced = odom_subscriber_->SyncedData(scan_time_stamp);
    //TODO
    //@IMU对齐
    // bool imu_synced = laser_subscriber_->SyncedData(scan_time_stamp);
    if(!odom_synced) {
        //若对齐不成功，将这一帧数据删掉
        laser_data.pop_front();
        return false;
    }
    return true;
}

bool DataPretreat::HasSyncedData() {
    if(laser_subscriber_->synced_data_.empty())
        return false;
    if(laser_subscriber_->synced_data_.empty())
        return false;
    return true;
}

bool DataPretreat::ValidData() {
    static std::deque<SensorPtr>& laser_data = laser_subscriber_->synced_data_;
    static std::deque<SensorPtr>& synced_odom_data = odom_subscriber_->synced_data_;

    synced_odom_ = CAST_TO_ODOM(synced_odom_data.front());
    distorted_laser_ = CAST_TO_SCAN(laser_data.front());

    double laser_timestamp = laser_data.front()->timestamp_;
    double odom_timestamp = synced_odom_data.front()->timestamp_;

    if(laser_timestamp - odom_timestamp > 0.05 ) {
        synced_odom_data.pop_front();
        return false;
    }

    if(laser_timestamp - odom_timestamp < -0.05) {
        laser_data.pop_front();
        return false;
    }

    laser_data.pop_front();
    synced_odom_data.pop_front();

}

void DataPretreat::Run() {
    if(!ReadData())
        return;
    while(HasSyncedData()) {
        if(!ValidData())
            continue;
        LaserDistortionRemove();

    }
}

void DataPretreat::Publish() {

    odom_publisher_->Publish(synced_odom_);
    laser_publisher_->Publish(undistorted_laser_);
}

void DataPretreat::LaserDistortionRemove() {
    //TODO
    //@ 激光雷达运动畸变的剔除
    undistorted_laser_ = distorted_laser_;
}


}
